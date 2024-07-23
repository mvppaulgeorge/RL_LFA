// Benchmark "adder" written by ABC on Thu Jul 18 04:28:08 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n318, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n329, new_n330, new_n333,
    new_n334, new_n336, new_n338;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[8] ), .o1(new_n99));
  orn002aa1n02x5               g004(.a(\a[8] ), .b(\b[7] ), .o(new_n100));
  nanp02aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(new_n100), .b(new_n101), .o1(new_n102));
  nor022aa1n08x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nand42aa1n03x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nanb02aa1n02x5               g009(.a(new_n103), .b(new_n104), .out0(new_n105));
  nor002aa1n03x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nand42aa1n16x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  norb02aa1n06x5               g012(.a(new_n107), .b(new_n106), .out0(new_n108));
  xorc02aa1n02x5               g013(.a(\a[5] ), .b(\b[4] ), .out0(new_n109));
  nona23aa1n03x5               g014(.a(new_n109), .b(new_n108), .c(new_n102), .d(new_n105), .out0(new_n110));
  nand42aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  xorc02aa1n02x5               g016(.a(\a[3] ), .b(\b[2] ), .out0(new_n112));
  inv000aa1d42x5               g017(.a(\a[2] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[1] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  nanp02aa1n09x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  aob012aa1n02x5               g021(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(new_n117), .b(new_n115), .o1(new_n118));
  oa0022aa1n24x5               g023(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  aoai13aa1n04x5               g025(.a(new_n111), .b(new_n120), .c(new_n118), .d(new_n112), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(new_n103), .b(new_n101), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[5] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\b[4] ), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n107), .b(new_n106), .c(new_n123), .d(new_n124), .o1(new_n125));
  norp03aa1n06x5               g030(.a(new_n125), .b(new_n102), .c(new_n105), .o1(new_n126));
  nano22aa1n06x5               g031(.a(new_n126), .b(new_n100), .c(new_n122), .out0(new_n127));
  tech160nm_fioai012aa1n04x5   g032(.a(new_n127), .b(new_n121), .c(new_n110), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(new_n98), .b(new_n99), .c(new_n128), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  nor042aa1n09x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oaih22aa1d12x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  oaib12aa1n12x5               g039(.a(new_n134), .b(new_n97), .c(\b[9] ), .out0(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  xroi22aa1d04x5               g041(.a(new_n97), .b(\b[9] ), .c(new_n99), .d(\a[9] ), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n133), .b(new_n136), .c(new_n128), .d(new_n137), .o1(new_n138));
  aoi112aa1n02x5               g043(.a(new_n136), .b(new_n133), .c(new_n128), .d(new_n137), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n131), .o1(new_n141));
  nor042aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  norp02aa1n02x5               g050(.a(\b[7] ), .b(\a[8] ), .o1(new_n146));
  nona23aa1n02x4               g051(.a(new_n104), .b(new_n101), .c(new_n146), .d(new_n103), .out0(new_n147));
  tech160nm_fixnrc02aa1n02p5x5 g052(.a(\b[4] ), .b(\a[5] ), .out0(new_n148));
  norb02aa1n03x5               g053(.a(new_n108), .b(new_n148), .out0(new_n149));
  inv000aa1n02x5               g054(.a(new_n111), .o1(new_n150));
  xnrc02aa1n02x5               g055(.a(\b[2] ), .b(\a[3] ), .out0(new_n151));
  aoai13aa1n12x5               g056(.a(new_n119), .b(new_n151), .c(new_n117), .d(new_n115), .o1(new_n152));
  nona23aa1d18x5               g057(.a(new_n152), .b(new_n149), .c(new_n147), .d(new_n150), .out0(new_n153));
  xorc02aa1n02x5               g058(.a(\a[10] ), .b(\b[9] ), .out0(new_n154));
  xorc02aa1n02x5               g059(.a(\a[9] ), .b(\b[8] ), .out0(new_n155));
  nano23aa1n06x5               g060(.a(new_n131), .b(new_n142), .c(new_n143), .d(new_n132), .out0(new_n156));
  nanp03aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n155), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n142), .b(new_n131), .c(new_n143), .o1(new_n158));
  aobi12aa1n02x5               g063(.a(new_n158), .b(new_n156), .c(new_n136), .out0(new_n159));
  aoai13aa1n06x5               g064(.a(new_n159), .b(new_n157), .c(new_n153), .d(new_n127), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanp02aa1n04x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n160), .c(new_n163), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n12x5               g070(.a(\b[14] ), .b(\a[15] ), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  norp02aa1n12x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nona23aa1d18x5               g074(.a(new_n169), .b(new_n163), .c(new_n162), .d(new_n168), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  oai012aa1n18x5               g076(.a(new_n169), .b(new_n168), .c(new_n162), .o1(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  aoai13aa1n04x5               g078(.a(new_n167), .b(new_n173), .c(new_n160), .d(new_n171), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n167), .b(new_n173), .c(new_n160), .d(new_n171), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  nor042aa1n06x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  xnrc02aa1n12x5               g082(.a(\b[15] ), .b(\a[16] ), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nona22aa1n02x4               g084(.a(new_n174), .b(new_n179), .c(new_n177), .out0(new_n180));
  inv000aa1d42x5               g085(.a(new_n177), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n178), .b(new_n174), .c(new_n181), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n180), .b(new_n182), .out0(\s[16] ));
  nor043aa1n04x5               g088(.a(new_n170), .b(new_n178), .c(new_n166), .o1(new_n184));
  nand23aa1n03x5               g089(.a(new_n184), .b(new_n137), .c(new_n156), .o1(new_n185));
  nona23aa1n02x4               g090(.a(new_n143), .b(new_n132), .c(new_n131), .d(new_n142), .out0(new_n186));
  oaih12aa1n02x5               g091(.a(new_n158), .b(new_n186), .c(new_n135), .o1(new_n187));
  oao003aa1n03x5               g092(.a(\a[16] ), .b(\b[15] ), .c(new_n181), .carry(new_n188));
  oai013aa1n03x4               g093(.a(new_n188), .b(new_n178), .c(new_n166), .d(new_n172), .o1(new_n189));
  aoi012aa1n06x5               g094(.a(new_n189), .b(new_n187), .c(new_n184), .o1(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n185), .c(new_n153), .d(new_n127), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[17] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[16] ), .o1(new_n194));
  tech160nm_fioaoi03aa1n03p5x5 g099(.a(new_n193), .b(new_n194), .c(new_n191), .o1(new_n195));
  xnrb03aa1n03x5               g100(.a(new_n195), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nanp02aa1n02x5               g101(.a(new_n194), .b(new_n193), .o1(new_n197));
  and002aa1n02x5               g102(.a(\b[16] ), .b(\a[17] ), .o(new_n198));
  nor042aa1n03x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nand22aa1n04x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nano23aa1n09x5               g105(.a(new_n199), .b(new_n198), .c(new_n197), .d(new_n200), .out0(new_n201));
  aoai13aa1n12x5               g106(.a(new_n200), .b(new_n199), .c(new_n193), .d(new_n194), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  nor042aa1n09x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand02aa1d04x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nanb02aa1d24x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n203), .c(new_n191), .d(new_n201), .o1(new_n208));
  aoi112aa1n02x5               g113(.a(new_n207), .b(new_n203), .c(new_n191), .d(new_n201), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g116(.a(\b[19] ), .o1(new_n212));
  nanb02aa1n06x5               g117(.a(\a[20] ), .b(new_n212), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nanp02aa1n06x5               g119(.a(new_n213), .b(new_n214), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  nona22aa1n02x5               g121(.a(new_n208), .b(new_n216), .c(new_n204), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n204), .o1(new_n218));
  tech160nm_fiaoi012aa1n05x5   g123(.a(new_n215), .b(new_n208), .c(new_n218), .o1(new_n219));
  norb02aa1n03x4               g124(.a(new_n217), .b(new_n219), .out0(\s[20] ));
  norp02aa1n02x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nona23aa1n08x5               g126(.a(new_n214), .b(new_n205), .c(new_n204), .d(new_n221), .out0(new_n222));
  inv030aa1n03x5               g127(.a(new_n222), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(new_n223), .b(new_n201), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(new_n204), .b(new_n214), .o1(new_n226));
  oai112aa1n06x5               g131(.a(new_n226), .b(new_n213), .c(new_n222), .d(new_n202), .o1(new_n227));
  xnrc02aa1n12x5               g132(.a(\b[20] ), .b(\a[21] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n227), .c(new_n191), .d(new_n225), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n227), .c(new_n191), .d(new_n225), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(\s[21] ));
  nor042aa1n03x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[21] ), .b(\a[22] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  nona22aa1n06x5               g140(.a(new_n230), .b(new_n235), .c(new_n233), .out0(new_n236));
  inv020aa1n02x5               g141(.a(new_n233), .o1(new_n237));
  tech160nm_fiaoi012aa1n05x5   g142(.a(new_n234), .b(new_n230), .c(new_n237), .o1(new_n238));
  norb02aa1n03x4               g143(.a(new_n236), .b(new_n238), .out0(\s[22] ));
  nor042aa1n06x5               g144(.a(new_n234), .b(new_n228), .o1(new_n240));
  oaoi03aa1n02x5               g145(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n227), .c(new_n240), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  nand23aa1n06x5               g148(.a(new_n223), .b(new_n240), .c(new_n201), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  tech160nm_fixorc02aa1n04x5   g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n243), .c(new_n191), .d(new_n245), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n243), .b(new_n246), .c(new_n191), .d(new_n245), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[23] ));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  tech160nm_fixorc02aa1n05x5   g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  nona22aa1n06x5               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n250), .o1(new_n253));
  aobi12aa1n06x5               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n03x4               g159(.a(new_n252), .b(new_n254), .out0(\s[24] ));
  nano32aa1n03x7               g160(.a(new_n224), .b(new_n251), .c(new_n240), .d(new_n246), .out0(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .out0(new_n257));
  norb02aa1n02x5               g162(.a(new_n251), .b(new_n257), .out0(new_n258));
  norp02aa1n02x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n260));
  nanp03aa1n02x5               g165(.a(new_n241), .b(new_n246), .c(new_n251), .o1(new_n261));
  nona22aa1n09x5               g166(.a(new_n261), .b(new_n260), .c(new_n259), .out0(new_n262));
  aoi013aa1n02x4               g167(.a(new_n262), .b(new_n227), .c(new_n240), .d(new_n258), .o1(new_n263));
  inv000aa1n02x5               g168(.a(new_n263), .o1(new_n264));
  tech160nm_fixorc02aa1n03p5x5 g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n264), .c(new_n191), .d(new_n256), .o1(new_n266));
  aoi112aa1n02x5               g171(.a(new_n265), .b(new_n264), .c(new_n191), .d(new_n256), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n266), .b(new_n267), .out0(\s[25] ));
  nor042aa1n03x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  tech160nm_fixorc02aa1n03p5x5 g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  nona22aa1n03x5               g175(.a(new_n266), .b(new_n270), .c(new_n269), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n269), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n270), .b(new_n266), .c(new_n272), .out0(new_n273));
  norb02aa1n03x4               g178(.a(new_n271), .b(new_n273), .out0(\s[26] ));
  nona22aa1n02x4               g179(.a(new_n171), .b(new_n178), .c(new_n166), .out0(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n157), .o1(new_n276));
  oabi12aa1n02x5               g181(.a(new_n189), .b(new_n159), .c(new_n275), .out0(new_n277));
  and002aa1n02x7               g182(.a(new_n270), .b(new_n265), .o(new_n278));
  nano22aa1n06x5               g183(.a(new_n244), .b(new_n278), .c(new_n258), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n277), .c(new_n128), .d(new_n276), .o1(new_n280));
  nor003aa1n03x5               g185(.a(new_n202), .b(new_n206), .c(new_n215), .o1(new_n281));
  nano22aa1n03x5               g186(.a(new_n281), .b(new_n213), .c(new_n226), .out0(new_n282));
  nano22aa1n03x7               g187(.a(new_n282), .b(new_n240), .c(new_n258), .out0(new_n283));
  oao003aa1n02x5               g188(.a(\a[26] ), .b(\b[25] ), .c(new_n272), .carry(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  oaoi13aa1n09x5               g190(.a(new_n285), .b(new_n278), .c(new_n283), .d(new_n262), .o1(new_n286));
  nor042aa1n03x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  norb02aa1n02x5               g193(.a(new_n288), .b(new_n287), .out0(new_n289));
  xnbna2aa1n03x5               g194(.a(new_n289), .b(new_n286), .c(new_n280), .out0(\s[27] ));
  inv000aa1d42x5               g195(.a(new_n287), .o1(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[27] ), .b(\a[28] ), .out0(new_n292));
  inv000aa1d42x5               g197(.a(new_n262), .o1(new_n293));
  nanp02aa1n02x5               g198(.a(new_n251), .b(new_n246), .o1(new_n294));
  nona32aa1n02x4               g199(.a(new_n227), .b(new_n294), .c(new_n234), .d(new_n228), .out0(new_n295));
  inv000aa1d42x5               g200(.a(new_n278), .o1(new_n296));
  aoai13aa1n06x5               g201(.a(new_n284), .b(new_n296), .c(new_n295), .d(new_n293), .o1(new_n297));
  aoai13aa1n02x5               g202(.a(new_n288), .b(new_n297), .c(new_n191), .d(new_n279), .o1(new_n298));
  aoi012aa1n03x5               g203(.a(new_n292), .b(new_n298), .c(new_n291), .o1(new_n299));
  aoi022aa1n02x7               g204(.a(new_n286), .b(new_n280), .c(\a[27] ), .d(\b[26] ), .o1(new_n300));
  nano22aa1n02x4               g205(.a(new_n300), .b(new_n291), .c(new_n292), .out0(new_n301));
  nor002aa1n02x5               g206(.a(new_n299), .b(new_n301), .o1(\s[28] ));
  nano22aa1n02x4               g207(.a(new_n292), .b(new_n291), .c(new_n288), .out0(new_n303));
  aoai13aa1n02x5               g208(.a(new_n303), .b(new_n297), .c(new_n191), .d(new_n279), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .c(new_n291), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[28] ), .b(\a[29] ), .out0(new_n306));
  aoi012aa1n03x5               g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n02x7               g212(.a(new_n303), .b(new_n286), .c(new_n280), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n02x5               g214(.a(new_n307), .b(new_n309), .o1(\s[29] ));
  xorb03aa1n02x5               g215(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g216(.a(new_n289), .b(new_n306), .c(new_n292), .out0(new_n312));
  aoai13aa1n02x5               g217(.a(new_n312), .b(new_n297), .c(new_n191), .d(new_n279), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[29] ), .b(\b[28] ), .c(new_n305), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[29] ), .b(\a[30] ), .out0(new_n315));
  aoi012aa1n03x5               g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  aobi12aa1n02x5               g221(.a(new_n312), .b(new_n286), .c(new_n280), .out0(new_n317));
  nano22aa1n03x5               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(new_n318));
  norp02aa1n02x5               g223(.a(new_n316), .b(new_n318), .o1(\s[30] ));
  xnrc02aa1n02x5               g224(.a(\b[30] ), .b(\a[31] ), .out0(new_n320));
  norb03aa1n02x5               g225(.a(new_n303), .b(new_n315), .c(new_n306), .out0(new_n321));
  aobi12aa1n02x5               g226(.a(new_n321), .b(new_n286), .c(new_n280), .out0(new_n322));
  oao003aa1n02x5               g227(.a(\a[30] ), .b(\b[29] ), .c(new_n314), .carry(new_n323));
  nano22aa1n02x4               g228(.a(new_n322), .b(new_n320), .c(new_n323), .out0(new_n324));
  aoai13aa1n02x5               g229(.a(new_n321), .b(new_n297), .c(new_n191), .d(new_n279), .o1(new_n325));
  aoi012aa1n03x5               g230(.a(new_n320), .b(new_n325), .c(new_n323), .o1(new_n326));
  norp02aa1n02x5               g231(.a(new_n326), .b(new_n324), .o1(\s[31] ));
  xnbna2aa1n03x5               g232(.a(new_n112), .b(new_n117), .c(new_n115), .out0(\s[3] ));
  oaoi03aa1n02x5               g233(.a(new_n113), .b(new_n114), .c(new_n116), .o1(new_n329));
  oaoi03aa1n02x5               g234(.a(\a[3] ), .b(\b[2] ), .c(new_n329), .o1(new_n330));
  xorb03aa1n02x5               g235(.a(new_n330), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g236(.a(new_n148), .b(new_n152), .c(new_n111), .out0(\s[5] ));
  nanp02aa1n02x5               g237(.a(\b[4] ), .b(\a[5] ), .o1(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n148), .c(new_n152), .d(new_n111), .o1(new_n334));
  xnrc02aa1n02x5               g239(.a(new_n334), .b(new_n108), .out0(\s[6] ));
  nanp02aa1n02x5               g240(.a(new_n334), .b(new_n108), .o1(new_n336));
  xnbna2aa1n03x5               g241(.a(new_n105), .b(new_n336), .c(new_n107), .out0(\s[7] ));
  aoi013aa1n02x4               g242(.a(new_n103), .b(new_n336), .c(new_n107), .d(new_n104), .o1(new_n338));
  xnbna2aa1n03x5               g243(.a(new_n338), .b(new_n100), .c(new_n101), .out0(\s[8] ));
  xnbna2aa1n03x5               g244(.a(new_n155), .b(new_n153), .c(new_n127), .out0(\s[9] ));
endmodule


