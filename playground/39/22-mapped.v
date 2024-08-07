// Benchmark "adder" written by ABC on Thu Jul 18 08:07:44 2024

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
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n338,
    new_n341, new_n343, new_n344;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oao003aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n102));
  nor002aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n02x4               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  tech160nm_fiao0012aa1n02p5x5 g012(.a(new_n103), .b(new_n105), .c(new_n104), .o(new_n108));
  aoi012aa1n02x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor002aa1n03x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand42aa1n10x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  norb02aa1n02x5               g016(.a(new_n111), .b(new_n110), .out0(new_n112));
  nor002aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  inv000aa1d42x5               g020(.a(\b[6] ), .o1(new_n116));
  nanb02aa1n02x5               g021(.a(\a[7] ), .b(new_n116), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(new_n117), .b(new_n118), .o1(new_n119));
  xorc02aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .out0(new_n120));
  nona23aa1n03x5               g025(.a(new_n120), .b(new_n112), .c(new_n115), .d(new_n119), .out0(new_n121));
  inv000aa1d42x5               g026(.a(new_n113), .o1(new_n122));
  aoi112aa1n06x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(new_n123), .o1(new_n124));
  oai022aa1n02x5               g029(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n125));
  nano23aa1n02x4               g030(.a(new_n115), .b(new_n119), .c(new_n125), .d(new_n111), .out0(new_n126));
  nano22aa1n03x7               g031(.a(new_n126), .b(new_n122), .c(new_n124), .out0(new_n127));
  oai012aa1n02x5               g032(.a(new_n127), .b(new_n109), .c(new_n121), .o1(new_n128));
  oaoi03aa1n02x5               g033(.a(new_n97), .b(new_n98), .c(new_n128), .o1(new_n129));
  xnrb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oaoi03aa1n02x5               g035(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n131));
  nona23aa1n02x4               g036(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n132));
  oabi12aa1n06x5               g037(.a(new_n108), .b(new_n132), .c(new_n131), .out0(new_n133));
  norb02aa1n03x5               g038(.a(new_n114), .b(new_n113), .out0(new_n134));
  nanp03aa1n02x5               g039(.a(new_n134), .b(new_n117), .c(new_n118), .o1(new_n135));
  nano22aa1n03x7               g040(.a(new_n135), .b(new_n120), .c(new_n112), .out0(new_n136));
  nanp02aa1n02x5               g041(.a(new_n133), .b(new_n136), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[8] ), .b(\a[9] ), .o1(new_n138));
  nand42aa1n02x5               g043(.a(\b[8] ), .b(\a[9] ), .o1(new_n139));
  nor002aa1n06x5               g044(.a(\b[9] ), .b(\a[10] ), .o1(new_n140));
  nand02aa1n03x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  nona23aa1n02x4               g046(.a(new_n141), .b(new_n139), .c(new_n138), .d(new_n140), .out0(new_n142));
  aoai13aa1n06x5               g047(.a(new_n141), .b(new_n140), .c(new_n97), .d(new_n98), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n137), .d(new_n127), .o1(new_n144));
  xorb03aa1n02x5               g049(.a(new_n144), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand02aa1d06x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  aoi012aa1n02x5               g052(.a(new_n146), .b(new_n144), .c(new_n147), .o1(new_n148));
  nor002aa1d32x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  nand22aa1n06x5               g055(.a(\b[11] ), .b(\a[12] ), .o1(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n148), .b(new_n151), .c(new_n150), .out0(\s[12] ));
  nand42aa1n03x5               g057(.a(new_n146), .b(new_n151), .o1(new_n153));
  nona23aa1d18x5               g058(.a(new_n151), .b(new_n147), .c(new_n146), .d(new_n149), .out0(new_n154));
  oai112aa1n06x5               g059(.a(new_n153), .b(new_n150), .c(new_n154), .d(new_n143), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nano23aa1n03x5               g061(.a(new_n138), .b(new_n140), .c(new_n141), .d(new_n139), .out0(new_n157));
  nano23aa1n03x7               g062(.a(new_n146), .b(new_n149), .c(new_n151), .d(new_n147), .out0(new_n158));
  nand02aa1n02x5               g063(.a(new_n158), .b(new_n157), .o1(new_n159));
  aoai13aa1n06x5               g064(.a(new_n156), .b(new_n159), .c(new_n137), .d(new_n127), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n04x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n160), .c(new_n163), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n03x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1n03x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  norb02aa1n02x7               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  norp02aa1n12x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand02aa1n06x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  tech160nm_fioai012aa1n05x5   g075(.a(new_n170), .b(new_n169), .c(new_n162), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  nano23aa1n06x5               g077(.a(new_n162), .b(new_n169), .c(new_n170), .d(new_n163), .out0(new_n173));
  aoai13aa1n04x5               g078(.a(new_n168), .b(new_n172), .c(new_n160), .d(new_n173), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n172), .b(new_n168), .c(new_n160), .d(new_n173), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n174), .b(new_n175), .out0(\s[15] ));
  norp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanp02aa1n02x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x7               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  nona22aa1n02x4               g084(.a(new_n174), .b(new_n179), .c(new_n166), .out0(new_n180));
  orn002aa1n02x5               g085(.a(\a[15] ), .b(\b[14] ), .o(new_n181));
  aobi12aa1n02x5               g086(.a(new_n179), .b(new_n174), .c(new_n181), .out0(new_n182));
  norb02aa1n02x5               g087(.a(new_n180), .b(new_n182), .out0(\s[16] ));
  inv000aa1d42x5               g088(.a(new_n111), .o1(new_n184));
  xorc02aa1n02x5               g089(.a(\a[7] ), .b(\b[6] ), .out0(new_n185));
  norp02aa1n02x5               g090(.a(\b[4] ), .b(\a[5] ), .o1(new_n186));
  norp02aa1n02x5               g091(.a(new_n186), .b(new_n110), .o1(new_n187));
  nona23aa1n02x4               g092(.a(new_n185), .b(new_n134), .c(new_n187), .d(new_n184), .out0(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(new_n123), .c(new_n113), .out0(new_n189));
  nano32aa1n03x7               g094(.a(new_n159), .b(new_n179), .c(new_n168), .d(new_n173), .out0(new_n190));
  aoai13aa1n09x5               g095(.a(new_n190), .b(new_n189), .c(new_n133), .d(new_n136), .o1(new_n191));
  nona23aa1n02x4               g096(.a(new_n170), .b(new_n163), .c(new_n162), .d(new_n169), .out0(new_n192));
  nona23aa1n03x5               g097(.a(new_n178), .b(new_n167), .c(new_n166), .d(new_n177), .out0(new_n193));
  nor042aa1n02x5               g098(.a(new_n193), .b(new_n192), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n195));
  oai022aa1n03x5               g100(.a(new_n193), .b(new_n171), .c(\b[15] ), .d(\a[16] ), .o1(new_n196));
  aoi112aa1n09x5               g101(.a(new_n196), .b(new_n195), .c(new_n155), .d(new_n194), .o1(new_n197));
  xorc02aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n197), .c(new_n191), .out0(\s[17] ));
  inv000aa1d42x5               g104(.a(\a[17] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(\b[16] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  nona22aa1n02x4               g107(.a(new_n194), .b(new_n154), .c(new_n142), .out0(new_n203));
  oaoi13aa1n09x5               g108(.a(new_n203), .b(new_n127), .c(new_n109), .d(new_n121), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n155), .b(new_n194), .o1(new_n205));
  nona22aa1n03x5               g110(.a(new_n205), .b(new_n196), .c(new_n195), .out0(new_n206));
  oai012aa1n02x5               g111(.a(new_n198), .b(new_n206), .c(new_n204), .o1(new_n207));
  nor022aa1n08x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nand42aa1n03x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nanb02aa1n03x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  xobna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n202), .out0(\s[18] ));
  nanp02aa1n02x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  nano22aa1n12x5               g117(.a(new_n210), .b(new_n202), .c(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n02x5               g119(.a(new_n209), .b(new_n208), .c(new_n200), .d(new_n201), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n214), .c(new_n197), .d(new_n191), .o1(new_n216));
  xorb03aa1n03x5               g121(.a(new_n216), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand02aa1d04x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nanb02aa1d24x5               g125(.a(new_n219), .b(new_n220), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  nor042aa1n06x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nanb02aa1n06x5               g129(.a(new_n223), .b(new_n224), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi112aa1n02x7               g131(.a(new_n219), .b(new_n226), .c(new_n216), .d(new_n222), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n219), .o1(new_n228));
  tech160nm_finand02aa1n03p5x5 g133(.a(new_n216), .b(new_n222), .o1(new_n229));
  aoi012aa1n03x5               g134(.a(new_n225), .b(new_n229), .c(new_n228), .o1(new_n230));
  norp02aa1n03x5               g135(.a(new_n230), .b(new_n227), .o1(\s[20] ));
  nona22aa1n02x4               g136(.a(new_n213), .b(new_n221), .c(new_n225), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n223), .o1(new_n233));
  nanp02aa1n02x5               g138(.a(new_n219), .b(new_n224), .o1(new_n234));
  nor003aa1n03x5               g139(.a(new_n215), .b(new_n221), .c(new_n225), .o1(new_n235));
  nano22aa1n03x7               g140(.a(new_n235), .b(new_n233), .c(new_n234), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n232), .c(new_n197), .d(new_n191), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  xnrc02aa1n12x5               g146(.a(\b[21] ), .b(\a[22] ), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoi112aa1n02x7               g148(.a(new_n239), .b(new_n243), .c(new_n237), .d(new_n241), .o1(new_n244));
  inv000aa1d42x5               g149(.a(new_n239), .o1(new_n245));
  tech160nm_finand02aa1n05x5   g150(.a(new_n237), .b(new_n241), .o1(new_n246));
  aoi012aa1n03x5               g151(.a(new_n242), .b(new_n246), .c(new_n245), .o1(new_n247));
  norp02aa1n03x5               g152(.a(new_n247), .b(new_n244), .o1(\s[22] ));
  nona23aa1n09x5               g153(.a(new_n224), .b(new_n220), .c(new_n219), .d(new_n223), .out0(new_n249));
  norp02aa1n06x5               g154(.a(new_n242), .b(new_n240), .o1(new_n250));
  nanb03aa1n06x5               g155(.a(new_n249), .b(new_n250), .c(new_n213), .out0(new_n251));
  oai112aa1n03x5               g156(.a(new_n234), .b(new_n233), .c(new_n249), .d(new_n215), .o1(new_n252));
  oaoi03aa1n02x5               g157(.a(\a[22] ), .b(\b[21] ), .c(new_n245), .o1(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n252), .c(new_n250), .o1(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n251), .c(new_n197), .d(new_n191), .o1(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  xorc02aa1n02x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  xorc02aa1n12x5               g163(.a(\a[24] ), .b(\b[23] ), .out0(new_n259));
  aoi112aa1n02x7               g164(.a(new_n257), .b(new_n259), .c(new_n255), .d(new_n258), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n257), .o1(new_n261));
  nand42aa1n02x5               g166(.a(new_n255), .b(new_n258), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n259), .o1(new_n263));
  aoi012aa1n03x5               g168(.a(new_n263), .b(new_n262), .c(new_n261), .o1(new_n264));
  norp02aa1n03x5               g169(.a(new_n264), .b(new_n260), .o1(\s[24] ));
  nanp02aa1n02x5               g170(.a(new_n259), .b(new_n258), .o1(new_n266));
  nona23aa1n02x4               g171(.a(new_n250), .b(new_n213), .c(new_n266), .d(new_n249), .out0(new_n267));
  xnrc02aa1n02x5               g172(.a(\b[22] ), .b(\a[23] ), .out0(new_n268));
  norb02aa1n02x5               g173(.a(new_n259), .b(new_n268), .out0(new_n269));
  norp02aa1n02x5               g174(.a(\b[23] ), .b(\a[24] ), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n271));
  nanp03aa1n03x5               g176(.a(new_n253), .b(new_n258), .c(new_n259), .o1(new_n272));
  nona22aa1n09x5               g177(.a(new_n272), .b(new_n271), .c(new_n270), .out0(new_n273));
  aoi013aa1n02x4               g178(.a(new_n273), .b(new_n252), .c(new_n250), .d(new_n269), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n267), .c(new_n197), .d(new_n191), .o1(new_n275));
  xorb03aa1n02x5               g180(.a(new_n275), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g181(.a(\b[24] ), .b(\a[25] ), .o1(new_n277));
  xorc02aa1n02x5               g182(.a(\a[25] ), .b(\b[24] ), .out0(new_n278));
  xorc02aa1n12x5               g183(.a(\a[26] ), .b(\b[25] ), .out0(new_n279));
  aoi112aa1n02x7               g184(.a(new_n277), .b(new_n279), .c(new_n275), .d(new_n278), .o1(new_n280));
  inv000aa1d42x5               g185(.a(new_n277), .o1(new_n281));
  nand42aa1n02x5               g186(.a(new_n275), .b(new_n278), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n279), .o1(new_n283));
  aoi012aa1n03x5               g188(.a(new_n283), .b(new_n282), .c(new_n281), .o1(new_n284));
  norp02aa1n03x5               g189(.a(new_n284), .b(new_n280), .o1(\s[26] ));
  and002aa1n02x5               g190(.a(new_n279), .b(new_n278), .o(new_n286));
  nano22aa1n03x7               g191(.a(new_n251), .b(new_n286), .c(new_n269), .out0(new_n287));
  oai012aa1n12x5               g192(.a(new_n287), .b(new_n206), .c(new_n204), .o1(new_n288));
  nano22aa1n03x7               g193(.a(new_n236), .b(new_n250), .c(new_n269), .out0(new_n289));
  oao003aa1n02x5               g194(.a(\a[26] ), .b(\b[25] ), .c(new_n281), .carry(new_n290));
  inv000aa1d42x5               g195(.a(new_n290), .o1(new_n291));
  oaoi13aa1n12x5               g196(.a(new_n291), .b(new_n286), .c(new_n289), .d(new_n273), .o1(new_n292));
  xorc02aa1n12x5               g197(.a(\a[27] ), .b(\b[26] ), .out0(new_n293));
  xnbna2aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n292), .out0(\s[27] ));
  norp02aa1n02x5               g199(.a(\b[26] ), .b(\a[27] ), .o1(new_n295));
  inv040aa1n03x5               g200(.a(new_n295), .o1(new_n296));
  inv000aa1d42x5               g201(.a(new_n293), .o1(new_n297));
  tech160nm_fiaoi012aa1n04x5   g202(.a(new_n297), .b(new_n288), .c(new_n292), .o1(new_n298));
  xnrc02aa1n12x5               g203(.a(\b[27] ), .b(\a[28] ), .out0(new_n299));
  nano22aa1n03x7               g204(.a(new_n298), .b(new_n296), .c(new_n299), .out0(new_n300));
  nand02aa1d06x5               g205(.a(new_n197), .b(new_n191), .o1(new_n301));
  nona32aa1n02x5               g206(.a(new_n252), .b(new_n266), .c(new_n242), .d(new_n240), .out0(new_n302));
  inv000aa1d42x5               g207(.a(new_n273), .o1(new_n303));
  inv000aa1n02x5               g208(.a(new_n286), .o1(new_n304));
  aoai13aa1n06x5               g209(.a(new_n290), .b(new_n304), .c(new_n302), .d(new_n303), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n293), .b(new_n305), .c(new_n301), .d(new_n287), .o1(new_n306));
  aoi012aa1n03x5               g211(.a(new_n299), .b(new_n306), .c(new_n296), .o1(new_n307));
  nor002aa1n02x5               g212(.a(new_n307), .b(new_n300), .o1(\s[28] ));
  norb02aa1d21x5               g213(.a(new_n293), .b(new_n299), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n305), .c(new_n301), .d(new_n287), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n296), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[28] ), .b(\a[29] ), .out0(new_n312));
  aoi012aa1n03x5               g217(.a(new_n312), .b(new_n310), .c(new_n311), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n309), .o1(new_n314));
  tech160nm_fiaoi012aa1n05x5   g219(.a(new_n314), .b(new_n288), .c(new_n292), .o1(new_n315));
  nano22aa1n03x7               g220(.a(new_n315), .b(new_n311), .c(new_n312), .out0(new_n316));
  nor002aa1n02x5               g221(.a(new_n313), .b(new_n316), .o1(\s[29] ));
  xorb03aa1n02x5               g222(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n03x5               g223(.a(new_n293), .b(new_n312), .c(new_n299), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n305), .c(new_n301), .d(new_n287), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[29] ), .b(\a[30] ), .out0(new_n322));
  aoi012aa1n02x7               g227(.a(new_n322), .b(new_n320), .c(new_n321), .o1(new_n323));
  inv000aa1d42x5               g228(.a(new_n319), .o1(new_n324));
  aoi012aa1n02x7               g229(.a(new_n324), .b(new_n288), .c(new_n292), .o1(new_n325));
  nano22aa1n03x7               g230(.a(new_n325), .b(new_n321), .c(new_n322), .out0(new_n326));
  nor002aa1n02x5               g231(.a(new_n323), .b(new_n326), .o1(\s[30] ));
  norb02aa1n09x5               g232(.a(new_n319), .b(new_n322), .out0(new_n328));
  inv000aa1n02x5               g233(.a(new_n328), .o1(new_n329));
  tech160nm_fiaoi012aa1n04x5   g234(.a(new_n329), .b(new_n288), .c(new_n292), .o1(new_n330));
  oao003aa1n02x5               g235(.a(\a[30] ), .b(\b[29] ), .c(new_n321), .carry(new_n331));
  xnrc02aa1n02x5               g236(.a(\b[30] ), .b(\a[31] ), .out0(new_n332));
  nano22aa1n03x7               g237(.a(new_n330), .b(new_n331), .c(new_n332), .out0(new_n333));
  aoai13aa1n03x5               g238(.a(new_n328), .b(new_n305), .c(new_n301), .d(new_n287), .o1(new_n334));
  aoi012aa1n03x5               g239(.a(new_n332), .b(new_n334), .c(new_n331), .o1(new_n335));
  nor002aa1n02x5               g240(.a(new_n335), .b(new_n333), .o1(\s[31] ));
  xnrb03aa1n02x5               g241(.a(new_n131), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g242(.a(\a[3] ), .b(\b[2] ), .c(new_n131), .o1(new_n338));
  xorb03aa1n02x5               g243(.a(new_n338), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g244(.a(new_n133), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g245(.a(\a[5] ), .b(\b[4] ), .c(new_n109), .o1(new_n341));
  xorb03aa1n02x5               g246(.a(new_n341), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g247(.a(new_n185), .b(new_n110), .c(new_n341), .d(new_n111), .o1(new_n343));
  aoi112aa1n02x5               g248(.a(new_n185), .b(new_n110), .c(new_n341), .d(new_n111), .o1(new_n344));
  norb02aa1n02x5               g249(.a(new_n343), .b(new_n344), .out0(\s[7] ));
  xnbna2aa1n03x5               g250(.a(new_n134), .b(new_n343), .c(new_n117), .out0(\s[8] ));
  xorb03aa1n02x5               g251(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


