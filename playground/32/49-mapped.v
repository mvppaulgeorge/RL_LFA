// Benchmark "adder" written by ABC on Thu Jul 18 04:49:02 2024

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
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n253, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n337, new_n338,
    new_n339, new_n341, new_n343, new_n345, new_n347;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nanp02aa1n02x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[6] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[5] ), .o1(new_n100));
  nor002aa1n03x5               g005(.a(\b[4] ), .b(\a[5] ), .o1(new_n101));
  aoi022aa1d24x5               g006(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n102));
  aoai13aa1n03x5               g007(.a(new_n102), .b(new_n101), .c(new_n99), .d(new_n100), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nor022aa1n04x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nor002aa1n02x5               g010(.a(new_n105), .b(new_n104), .o1(new_n106));
  aob012aa1n02x5               g011(.a(new_n98), .b(new_n103), .c(new_n106), .out0(new_n107));
  nor002aa1d32x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  norb02aa1n06x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  orn002aa1n03x5               g015(.a(\a[2] ), .b(\b[1] ), .o(new_n111));
  nanp02aa1n04x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  aob012aa1n06x5               g017(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(new_n113));
  nanp02aa1n04x5               g018(.a(new_n113), .b(new_n111), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\a[4] ), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\b[3] ), .o1(new_n116));
  aoi012aa1n02x7               g021(.a(new_n108), .b(new_n115), .c(new_n116), .o1(new_n117));
  inv000aa1n02x5               g022(.a(new_n117), .o1(new_n118));
  tech160nm_fiaoi012aa1n05x5   g023(.a(new_n118), .b(new_n114), .c(new_n110), .o1(new_n119));
  and002aa1n24x5               g024(.a(\b[3] ), .b(\a[4] ), .o(new_n120));
  norb03aa1n03x5               g025(.a(new_n98), .b(new_n120), .c(new_n104), .out0(new_n121));
  xorc02aa1n02x5               g026(.a(\a[6] ), .b(\b[5] ), .out0(new_n122));
  nand42aa1n03x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  nand42aa1n03x5               g028(.a(\b[4] ), .b(\a[5] ), .o1(new_n124));
  nano23aa1n02x5               g029(.a(new_n101), .b(new_n105), .c(new_n124), .d(new_n123), .out0(new_n125));
  nand23aa1n03x5               g030(.a(new_n125), .b(new_n121), .c(new_n122), .o1(new_n126));
  oai012aa1n12x5               g031(.a(new_n107), .b(new_n126), .c(new_n119), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aob012aa1n02x5               g033(.a(new_n97), .b(new_n127), .c(new_n128), .out0(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1d42x5               g035(.a(new_n128), .o1(new_n131));
  oaoi13aa1n02x5               g036(.a(new_n131), .b(new_n107), .c(new_n126), .d(new_n119), .o1(new_n132));
  xorc02aa1n02x5               g037(.a(\a[10] ), .b(\b[9] ), .out0(new_n133));
  nor042aa1n09x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand22aa1n03x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  tech160nm_fioaoi03aa1n03p5x5 g041(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n137), .c(new_n132), .d(new_n133), .o1(new_n138));
  aoi113aa1n02x5               g043(.a(new_n136), .b(new_n137), .c(new_n127), .d(new_n128), .e(new_n133), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n138), .b(new_n139), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n134), .o1(new_n141));
  nor042aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand02aa1n03x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  xnbna2aa1n03x5               g049(.a(new_n144), .b(new_n138), .c(new_n141), .out0(\s[12] ));
  aoi022aa1n02x5               g050(.a(new_n103), .b(new_n106), .c(\b[7] ), .d(\a[8] ), .o1(new_n146));
  nanb02aa1n02x5               g051(.a(new_n108), .b(new_n109), .out0(new_n147));
  aoai13aa1n02x5               g052(.a(new_n117), .b(new_n147), .c(new_n113), .d(new_n111), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n104), .o1(new_n149));
  oai112aa1n02x5               g054(.a(new_n149), .b(new_n98), .c(new_n116), .d(new_n115), .o1(new_n150));
  xnrc02aa1n02x5               g055(.a(\b[5] ), .b(\a[6] ), .out0(new_n151));
  nona23aa1n02x4               g056(.a(new_n124), .b(new_n123), .c(new_n101), .d(new_n105), .out0(new_n152));
  norp03aa1n03x5               g057(.a(new_n152), .b(new_n151), .c(new_n150), .o1(new_n153));
  nano23aa1n06x5               g058(.a(new_n134), .b(new_n142), .c(new_n143), .d(new_n135), .out0(new_n154));
  and003aa1n02x5               g059(.a(new_n154), .b(new_n133), .c(new_n128), .o(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n146), .c(new_n153), .d(new_n148), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n142), .b(new_n134), .c(new_n143), .o1(new_n157));
  aobi12aa1n06x5               g062(.a(new_n157), .b(new_n154), .c(new_n137), .out0(new_n158));
  nor002aa1n02x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  tech160nm_finand02aa1n03p5x5 g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n159), .b(new_n160), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n156), .c(new_n158), .out0(\s[13] ));
  inv040aa1d28x5               g067(.a(\a[13] ), .o1(new_n163));
  inv040aa1d32x5               g068(.a(\b[12] ), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(new_n156), .b(new_n158), .o1(new_n165));
  oaoi03aa1n02x5               g070(.a(new_n163), .b(new_n164), .c(new_n165), .o1(new_n166));
  xnrb03aa1n02x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n12x5               g072(.a(\b[14] ), .b(\a[15] ), .out0(new_n168));
  nor002aa1d32x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1d28x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  aoai13aa1n12x5               g075(.a(new_n170), .b(new_n169), .c(new_n163), .d(new_n164), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n169), .b(new_n170), .out0(new_n172));
  nona22aa1n02x4               g077(.a(new_n165), .b(new_n161), .c(new_n172), .out0(new_n173));
  xobna2aa1n03x5               g078(.a(new_n168), .b(new_n173), .c(new_n171), .out0(\s[15] ));
  orn002aa1n02x5               g079(.a(\a[15] ), .b(\b[14] ), .o(new_n175));
  tech160nm_fiaoi012aa1n03p5x5 g080(.a(new_n168), .b(new_n173), .c(new_n171), .o1(new_n176));
  tech160nm_fixnrc02aa1n02p5x5 g081(.a(\b[15] ), .b(\a[16] ), .out0(new_n177));
  nano22aa1n02x4               g082(.a(new_n176), .b(new_n175), .c(new_n177), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n171), .o1(new_n179));
  aoi112aa1n03x4               g084(.a(new_n172), .b(new_n161), .c(new_n156), .d(new_n158), .o1(new_n180));
  oabi12aa1n02x5               g085(.a(new_n168), .b(new_n180), .c(new_n179), .out0(new_n181));
  tech160nm_fiaoi012aa1n02p5x5 g086(.a(new_n177), .b(new_n181), .c(new_n175), .o1(new_n182));
  norp02aa1n02x5               g087(.a(new_n182), .b(new_n178), .o1(\s[16] ));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  nano23aa1n03x5               g089(.a(new_n159), .b(new_n169), .c(new_n170), .d(new_n160), .out0(new_n185));
  nona22aa1n03x5               g090(.a(new_n185), .b(new_n177), .c(new_n168), .out0(new_n186));
  nano32aa1n03x7               g091(.a(new_n186), .b(new_n154), .c(new_n133), .d(new_n128), .out0(new_n187));
  inv000aa1d42x5               g092(.a(\a[16] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[15] ), .o1(new_n189));
  nand02aa1d06x5               g094(.a(new_n171), .b(new_n175), .o1(new_n190));
  aoi022aa1n02x5               g095(.a(\b[15] ), .b(\a[16] ), .c(\a[15] ), .d(\b[14] ), .o1(new_n191));
  aoi022aa1n12x5               g096(.a(new_n190), .b(new_n191), .c(new_n189), .d(new_n188), .o1(new_n192));
  oai012aa1n12x5               g097(.a(new_n192), .b(new_n158), .c(new_n186), .o1(new_n193));
  aoi012aa1n12x5               g098(.a(new_n193), .b(new_n127), .c(new_n187), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(new_n184), .out0(\s[17] ));
  inv000aa1d42x5               g100(.a(\b[16] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(new_n196), .b(new_n184), .o1(new_n197));
  xorc02aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  aoai13aa1n06x5               g103(.a(new_n198), .b(new_n193), .c(new_n127), .d(new_n187), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[18] ), .b(\b[17] ), .out0(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n199), .c(new_n197), .out0(\s[18] ));
  and002aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o(new_n202));
  inv030aa1d32x5               g107(.a(\a[19] ), .o1(new_n203));
  inv020aa1d32x5               g108(.a(\b[18] ), .o1(new_n204));
  nand02aa1n04x5               g109(.a(new_n204), .b(new_n203), .o1(new_n205));
  nand02aa1n03x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(new_n205), .b(new_n206), .o1(new_n207));
  oai022aa1d24x5               g112(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoi112aa1n03x5               g114(.a(new_n207), .b(new_n202), .c(new_n199), .d(new_n209), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n207), .b(new_n202), .c(new_n199), .d(new_n209), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n211), .b(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand02aa1d28x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  nanb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  nano22aa1n02x4               g121(.a(new_n210), .b(new_n205), .c(new_n216), .out0(new_n217));
  nanp02aa1n03x5               g122(.a(new_n199), .b(new_n209), .o1(new_n218));
  nona22aa1n03x5               g123(.a(new_n218), .b(new_n207), .c(new_n202), .out0(new_n219));
  tech160nm_fiaoi012aa1n02p5x5 g124(.a(new_n216), .b(new_n219), .c(new_n205), .o1(new_n220));
  norp02aa1n03x5               g125(.a(new_n220), .b(new_n217), .o1(\s[20] ));
  nano22aa1n03x7               g126(.a(new_n214), .b(new_n206), .c(new_n215), .out0(new_n222));
  nanp02aa1n02x5               g127(.a(new_n198), .b(new_n205), .o1(new_n223));
  nano22aa1n03x7               g128(.a(new_n223), .b(new_n200), .c(new_n222), .out0(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n193), .c(new_n127), .d(new_n187), .o1(new_n225));
  nanb03aa1n06x5               g130(.a(new_n214), .b(new_n215), .c(new_n206), .out0(new_n226));
  inv000aa1d42x5               g131(.a(\a[18] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[17] ), .o1(new_n228));
  oai112aa1n06x5               g133(.a(new_n208), .b(new_n205), .c(new_n228), .d(new_n227), .o1(new_n229));
  aoi013aa1n09x5               g134(.a(new_n214), .b(new_n215), .c(new_n203), .d(new_n204), .o1(new_n230));
  oai012aa1n18x5               g135(.a(new_n230), .b(new_n229), .c(new_n226), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[20] ), .b(\a[21] ), .out0(new_n233));
  xobna2aa1n03x5               g138(.a(new_n233), .b(new_n225), .c(new_n232), .out0(\s[21] ));
  nor002aa1d32x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n236), .b(new_n233), .c(new_n225), .d(new_n232), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[21] ), .c(\a[22] ), .out0(\s[22] ));
  nand02aa1d04x5               g143(.a(new_n127), .b(new_n187), .o1(new_n239));
  inv040aa1n03x5               g144(.a(new_n192), .o1(new_n240));
  oab012aa1n02x5               g145(.a(new_n240), .b(new_n158), .c(new_n186), .out0(new_n241));
  nanp02aa1n06x5               g146(.a(new_n239), .b(new_n241), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n224), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[21] ), .b(\a[22] ), .out0(new_n244));
  nona32aa1n03x5               g149(.a(new_n242), .b(new_n244), .c(new_n233), .d(new_n243), .out0(new_n245));
  nor042aa1n12x5               g150(.a(new_n244), .b(new_n233), .o1(new_n246));
  oao003aa1n09x5               g151(.a(\a[22] ), .b(\b[21] ), .c(new_n236), .carry(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoi012aa1d18x5               g153(.a(new_n248), .b(new_n231), .c(new_n246), .o1(new_n249));
  xnrc02aa1n12x5               g154(.a(\b[22] ), .b(\a[23] ), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n250), .o1(new_n251));
  xnbna2aa1n03x5               g156(.a(new_n251), .b(new_n245), .c(new_n249), .out0(\s[23] ));
  nor042aa1n06x5               g157(.a(\b[22] ), .b(\a[23] ), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  tech160nm_fiaoi012aa1n05x5   g159(.a(new_n250), .b(new_n245), .c(new_n249), .o1(new_n255));
  tech160nm_fixnrc02aa1n04x5   g160(.a(\b[23] ), .b(\a[24] ), .out0(new_n256));
  nano22aa1n03x7               g161(.a(new_n255), .b(new_n254), .c(new_n256), .out0(new_n257));
  nano22aa1n03x7               g162(.a(new_n194), .b(new_n224), .c(new_n246), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n249), .o1(new_n259));
  tech160nm_fioai012aa1n05x5   g164(.a(new_n251), .b(new_n258), .c(new_n259), .o1(new_n260));
  tech160nm_fiaoi012aa1n04x5   g165(.a(new_n256), .b(new_n260), .c(new_n254), .o1(new_n261));
  norp02aa1n03x5               g166(.a(new_n261), .b(new_n257), .o1(\s[24] ));
  nor042aa1n04x5               g167(.a(new_n256), .b(new_n250), .o1(new_n263));
  and002aa1n02x5               g168(.a(new_n263), .b(new_n246), .o(new_n264));
  inv000aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  oaih22aa1d12x5               g170(.a(new_n227), .b(new_n228), .c(\b[18] ), .d(\a[19] ), .o1(new_n266));
  norb02aa1n06x5               g171(.a(new_n208), .b(new_n266), .out0(new_n267));
  inv020aa1n02x5               g172(.a(new_n230), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n246), .b(new_n268), .c(new_n267), .d(new_n222), .o1(new_n269));
  inv030aa1n02x5               g174(.a(new_n263), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[24] ), .b(\b[23] ), .c(new_n254), .carry(new_n271));
  aoai13aa1n12x5               g176(.a(new_n271), .b(new_n270), .c(new_n269), .d(new_n247), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xnrc02aa1n12x5               g178(.a(\b[24] ), .b(\a[25] ), .out0(new_n274));
  oaoi13aa1n09x5               g179(.a(new_n274), .b(new_n273), .c(new_n225), .d(new_n265), .o1(new_n275));
  nano22aa1n03x7               g180(.a(new_n194), .b(new_n224), .c(new_n264), .out0(new_n276));
  nano22aa1n02x4               g181(.a(new_n276), .b(new_n273), .c(new_n274), .out0(new_n277));
  norp02aa1n02x5               g182(.a(new_n275), .b(new_n277), .o1(\s[25] ));
  nor042aa1n03x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[25] ), .b(\a[26] ), .out0(new_n281));
  nano22aa1n03x5               g186(.a(new_n275), .b(new_n280), .c(new_n281), .out0(new_n282));
  inv000aa1d42x5               g187(.a(new_n274), .o1(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n276), .c(new_n272), .o1(new_n284));
  aoi012aa1n03x5               g189(.a(new_n281), .b(new_n284), .c(new_n280), .o1(new_n285));
  nor002aa1n02x5               g190(.a(new_n285), .b(new_n282), .o1(\s[26] ));
  nor042aa1n04x5               g191(.a(new_n281), .b(new_n274), .o1(new_n287));
  inv000aa1n02x5               g192(.a(new_n287), .o1(new_n288));
  nona32aa1n09x5               g193(.a(new_n242), .b(new_n288), .c(new_n265), .d(new_n243), .out0(new_n289));
  oao003aa1n02x5               g194(.a(\a[26] ), .b(\b[25] ), .c(new_n280), .carry(new_n290));
  aobi12aa1n12x5               g195(.a(new_n290), .b(new_n272), .c(new_n287), .out0(new_n291));
  nor042aa1n03x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  and002aa1n12x5               g197(.a(\b[26] ), .b(\a[27] ), .o(new_n293));
  norp02aa1n02x5               g198(.a(new_n293), .b(new_n292), .o1(new_n294));
  xnbna2aa1n03x5               g199(.a(new_n294), .b(new_n289), .c(new_n291), .out0(\s[27] ));
  inv000aa1d42x5               g200(.a(new_n292), .o1(new_n296));
  xnrc02aa1n12x5               g201(.a(\b[27] ), .b(\a[28] ), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n288), .b(new_n246), .c(new_n263), .out0(new_n298));
  nano22aa1n03x7               g203(.a(new_n194), .b(new_n224), .c(new_n298), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n263), .b(new_n248), .c(new_n231), .d(new_n246), .o1(new_n300));
  aoai13aa1n04x5               g205(.a(new_n290), .b(new_n288), .c(new_n300), .d(new_n271), .o1(new_n301));
  inv000aa1d42x5               g206(.a(new_n293), .o1(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n299), .c(new_n301), .o1(new_n303));
  aoi012aa1n03x5               g208(.a(new_n297), .b(new_n303), .c(new_n296), .o1(new_n304));
  aoi022aa1n03x5               g209(.a(new_n289), .b(new_n291), .c(\b[26] ), .d(\a[27] ), .o1(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n296), .c(new_n297), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[28] ));
  nano22aa1n12x5               g212(.a(new_n297), .b(new_n302), .c(new_n296), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n308), .b(new_n299), .c(new_n301), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[28] ), .b(\b[27] ), .c(new_n296), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[28] ), .b(\a[29] ), .out0(new_n311));
  aoi012aa1n03x5               g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n308), .o1(new_n313));
  tech160nm_fiaoi012aa1n05x5   g218(.a(new_n313), .b(new_n289), .c(new_n291), .o1(new_n314));
  nano22aa1n03x7               g219(.a(new_n314), .b(new_n310), .c(new_n311), .out0(new_n315));
  nor002aa1n02x5               g220(.a(new_n312), .b(new_n315), .o1(\s[29] ));
  xorb03aa1n02x5               g221(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g222(.a(new_n294), .b(new_n311), .c(new_n297), .out0(new_n318));
  oaih12aa1n02x5               g223(.a(new_n318), .b(new_n299), .c(new_n301), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .carry(new_n320));
  xnrc02aa1n02x5               g225(.a(\b[29] ), .b(\a[30] ), .out0(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n321), .b(new_n319), .c(new_n320), .o1(new_n322));
  inv000aa1d42x5               g227(.a(new_n318), .o1(new_n323));
  tech160nm_fiaoi012aa1n05x5   g228(.a(new_n323), .b(new_n289), .c(new_n291), .o1(new_n324));
  nano22aa1n03x7               g229(.a(new_n324), .b(new_n320), .c(new_n321), .out0(new_n325));
  nor002aa1n02x5               g230(.a(new_n322), .b(new_n325), .o1(\s[30] ));
  norb03aa1n02x5               g231(.a(new_n308), .b(new_n321), .c(new_n311), .out0(new_n327));
  inv000aa1n02x5               g232(.a(new_n327), .o1(new_n328));
  tech160nm_fiaoi012aa1n05x5   g233(.a(new_n328), .b(new_n289), .c(new_n291), .o1(new_n329));
  oao003aa1n02x5               g234(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n330));
  xnrc02aa1n02x5               g235(.a(\b[30] ), .b(\a[31] ), .out0(new_n331));
  nano22aa1n03x7               g236(.a(new_n329), .b(new_n330), .c(new_n331), .out0(new_n332));
  oaih12aa1n02x5               g237(.a(new_n327), .b(new_n299), .c(new_n301), .o1(new_n333));
  tech160nm_fiaoi012aa1n02p5x5 g238(.a(new_n331), .b(new_n333), .c(new_n330), .o1(new_n334));
  nor002aa1n02x5               g239(.a(new_n334), .b(new_n332), .o1(\s[31] ));
  xnbna2aa1n03x5               g240(.a(new_n110), .b(new_n113), .c(new_n111), .out0(\s[3] ));
  nanp02aa1n02x5               g241(.a(new_n116), .b(new_n115), .o1(new_n337));
  inv000aa1d42x5               g242(.a(new_n120), .o1(new_n338));
  aoi122aa1n02x5               g243(.a(new_n108), .b(new_n338), .c(new_n337), .d(new_n114), .e(new_n109), .o1(new_n339));
  aoi013aa1n02x4               g244(.a(new_n339), .b(new_n148), .c(new_n338), .d(new_n337), .o1(\s[4] ));
  aoai13aa1n02x5               g245(.a(new_n338), .b(new_n118), .c(new_n114), .d(new_n110), .o1(new_n341));
  xnrb03aa1n02x5               g246(.a(new_n341), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g247(.a(\a[5] ), .b(\b[4] ), .c(new_n341), .o1(new_n343));
  xorb03aa1n02x5               g248(.a(new_n343), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g249(.a(new_n99), .b(new_n100), .c(new_n343), .o1(new_n345));
  xnrb03aa1n02x5               g250(.a(new_n345), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g251(.a(\a[7] ), .b(\b[6] ), .c(new_n345), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g253(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


