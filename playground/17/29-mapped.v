// Benchmark "adder" written by ABC on Wed Jul 17 20:55:33 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n252, new_n253, new_n254, new_n255, new_n256, new_n257, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n318, new_n319, new_n320, new_n322, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n330, new_n332, new_n333, new_n334,
    new_n335, new_n337, new_n338, new_n340;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n16x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n04x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  nand42aa1d28x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor002aa1d32x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanb03aa1n06x5               g006(.a(new_n101), .b(new_n99), .c(new_n100), .out0(new_n102));
  nand22aa1n09x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  norp02aa1n04x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  norb03aa1n03x5               g009(.a(new_n100), .b(new_n104), .c(new_n103), .out0(new_n105));
  tech160nm_fixnrc02aa1n05x5   g010(.a(\b[3] ), .b(\a[4] ), .out0(new_n106));
  nor042aa1n02x5               g011(.a(new_n106), .b(new_n101), .o1(new_n107));
  oaih12aa1n06x5               g012(.a(new_n107), .b(new_n105), .c(new_n102), .o1(new_n108));
  aoi022aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n109));
  tech160nm_fixnrc02aa1n02p5x5 g014(.a(\b[6] ), .b(\a[7] ), .out0(new_n110));
  nor022aa1n16x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  aoi012aa1n02x5               g016(.a(new_n111), .b(\a[5] ), .c(\b[4] ), .o1(new_n112));
  inv040aa1d32x5               g017(.a(\a[8] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[7] ), .o1(new_n114));
  nand42aa1n06x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  nand42aa1n06x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  oai112aa1n03x5               g021(.a(new_n115), .b(new_n116), .c(\b[4] ), .d(\a[5] ), .o1(new_n117));
  nano23aa1n06x5               g022(.a(new_n117), .b(new_n110), .c(new_n112), .d(new_n109), .out0(new_n118));
  nor042aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano22aa1n06x5               g024(.a(new_n119), .b(new_n115), .c(new_n116), .out0(new_n120));
  inv000aa1d42x5               g025(.a(\a[7] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[6] ), .o1(new_n122));
  oai112aa1n02x5               g027(.a(new_n115), .b(new_n116), .c(new_n122), .d(new_n121), .o1(new_n123));
  nanp02aa1n04x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  nor002aa1d32x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  oai122aa1n06x5               g030(.a(new_n124), .b(new_n111), .c(new_n125), .d(\b[6] ), .e(\a[7] ), .o1(new_n126));
  obai22aa1n06x5               g031(.a(new_n116), .b(new_n120), .c(new_n123), .d(new_n126), .out0(new_n127));
  nand42aa1n08x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n97), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n127), .c(new_n108), .d(new_n118), .o1(new_n130));
  nor002aa1n03x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  oai022aa1d24x5               g038(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n134));
  nanb03aa1n02x5               g039(.a(new_n134), .b(new_n130), .c(new_n132), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n133), .c(new_n98), .d(new_n130), .o1(\s[10] ));
  nano23aa1n09x5               g041(.a(new_n97), .b(new_n131), .c(new_n132), .d(new_n128), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n127), .c(new_n108), .d(new_n118), .o1(new_n138));
  oaoi03aa1n02x5               g043(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nor042aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nand42aa1n16x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n138), .c(new_n140), .out0(\s[11] ));
  inv040aa1d32x5               g049(.a(\a[11] ), .o1(new_n145));
  inv040aa1d28x5               g050(.a(\b[10] ), .o1(new_n146));
  nand02aa1d06x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  aob012aa1n02x5               g052(.a(new_n143), .b(new_n138), .c(new_n140), .out0(new_n148));
  nor002aa1d32x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanp02aa1n24x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  nona23aa1n02x4               g056(.a(new_n148), .b(new_n150), .c(new_n149), .d(new_n141), .out0(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n151), .c(new_n147), .d(new_n148), .o1(\s[12] ));
  nano23aa1n09x5               g058(.a(new_n141), .b(new_n149), .c(new_n150), .d(new_n142), .out0(new_n154));
  nand22aa1n12x5               g059(.a(new_n154), .b(new_n137), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n127), .c(new_n108), .d(new_n118), .o1(new_n157));
  nanb03aa1d24x5               g062(.a(new_n149), .b(new_n150), .c(new_n142), .out0(new_n158));
  nand23aa1n09x5               g063(.a(new_n134), .b(new_n147), .c(new_n132), .o1(new_n159));
  aoai13aa1n12x5               g064(.a(new_n150), .b(new_n149), .c(new_n145), .d(new_n146), .o1(new_n160));
  oai012aa1d24x5               g065(.a(new_n160), .b(new_n159), .c(new_n158), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nor042aa1n12x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  nand42aa1n10x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n157), .c(new_n162), .out0(\s[13] ));
  inv000aa1d42x5               g071(.a(new_n163), .o1(new_n167));
  aob012aa1n02x5               g072(.a(new_n165), .b(new_n157), .c(new_n162), .out0(new_n168));
  nor042aa1n04x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1n08x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  nona23aa1n02x4               g076(.a(new_n168), .b(new_n170), .c(new_n169), .d(new_n163), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n171), .c(new_n167), .d(new_n168), .o1(\s[14] ));
  nano23aa1d15x5               g078(.a(new_n163), .b(new_n169), .c(new_n170), .d(new_n164), .out0(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  oaoi03aa1n02x5               g080(.a(\a[14] ), .b(\b[13] ), .c(new_n167), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n176), .b(new_n161), .c(new_n174), .o1(new_n177));
  tech160nm_fioai012aa1n03p5x5 g082(.a(new_n177), .b(new_n157), .c(new_n175), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nand42aa1n06x5               g085(.a(\b[14] ), .b(\a[15] ), .o1(new_n181));
  nano22aa1n02x4               g086(.a(new_n180), .b(new_n178), .c(new_n181), .out0(new_n182));
  aoi012aa1n02x5               g087(.a(new_n180), .b(new_n178), .c(new_n181), .o1(new_n183));
  nor042aa1n02x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  nanp02aa1n04x5               g089(.a(\b[15] ), .b(\a[16] ), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  nona22aa1n02x4               g091(.a(new_n185), .b(new_n184), .c(new_n180), .out0(new_n187));
  oai022aa1n02x5               g092(.a(new_n182), .b(new_n187), .c(new_n186), .d(new_n183), .o1(\s[16] ));
  nano23aa1n03x7               g093(.a(new_n180), .b(new_n184), .c(new_n185), .d(new_n181), .out0(new_n189));
  aoai13aa1n09x5               g094(.a(new_n189), .b(new_n176), .c(new_n161), .d(new_n174), .o1(new_n190));
  nano22aa1n03x7               g095(.a(new_n155), .b(new_n174), .c(new_n189), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n127), .c(new_n108), .d(new_n118), .o1(new_n192));
  oai012aa1n02x5               g097(.a(new_n185), .b(new_n184), .c(new_n180), .o1(new_n193));
  nand23aa1d12x5               g098(.a(new_n192), .b(new_n190), .c(new_n193), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  nano32aa1n02x4               g100(.a(new_n195), .b(new_n192), .c(new_n193), .d(new_n190), .out0(new_n196));
  aoi012aa1n02x5               g101(.a(new_n196), .b(new_n194), .c(new_n195), .o1(\s[17] ));
  nor042aa1n03x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  nand22aa1n03x5               g104(.a(new_n194), .b(new_n195), .o1(new_n200));
  xorc02aa1n02x5               g105(.a(\a[18] ), .b(\b[17] ), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  oai022aa1n04x7               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  nanb03aa1n02x5               g108(.a(new_n203), .b(new_n200), .c(new_n202), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n201), .c(new_n200), .d(new_n199), .o1(\s[18] ));
  xnrc02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .out0(new_n206));
  norb02aa1n06x5               g111(.a(new_n195), .b(new_n206), .out0(new_n207));
  oaoi03aa1n02x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n208));
  xorc02aa1n02x5               g113(.a(\a[19] ), .b(\b[18] ), .out0(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n208), .c(new_n194), .d(new_n207), .o1(new_n210));
  aoi112aa1n02x5               g115(.a(new_n209), .b(new_n208), .c(new_n194), .d(new_n207), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g118(.a(\a[19] ), .o1(new_n214));
  inv000aa1d42x5               g119(.a(\b[18] ), .o1(new_n215));
  nanp02aa1n02x5               g120(.a(new_n215), .b(new_n214), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[20] ), .b(\b[19] ), .out0(new_n217));
  nand02aa1d10x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  oai022aa1n02x5               g124(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n220));
  nona22aa1n02x5               g125(.a(new_n210), .b(new_n219), .c(new_n220), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n217), .c(new_n216), .d(new_n210), .o1(\s[20] ));
  inv040aa1d32x5               g127(.a(\a[20] ), .o1(new_n223));
  xroi22aa1d06x4               g128(.a(new_n214), .b(\b[18] ), .c(new_n223), .d(\b[19] ), .out0(new_n224));
  nand22aa1n06x5               g129(.a(new_n207), .b(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  nanb02aa1n02x5               g131(.a(\b[19] ), .b(new_n223), .out0(new_n227));
  oai112aa1n04x5               g132(.a(new_n227), .b(new_n218), .c(new_n215), .d(new_n214), .o1(new_n228));
  nanp03aa1n02x5               g133(.a(new_n203), .b(new_n216), .c(new_n202), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n220), .b(new_n218), .o1(new_n230));
  oai012aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n228), .o1(new_n231));
  xorc02aa1n02x5               g136(.a(\a[21] ), .b(\b[20] ), .out0(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(new_n194), .d(new_n226), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n232), .b(new_n231), .c(new_n194), .d(new_n226), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n233), .b(new_n234), .out0(\s[21] ));
  inv000aa1d42x5               g140(.a(\a[21] ), .o1(new_n236));
  nanb02aa1n02x5               g141(.a(\b[20] ), .b(new_n236), .out0(new_n237));
  xorc02aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .out0(new_n238));
  and002aa1n02x5               g143(.a(\b[21] ), .b(\a[22] ), .o(new_n239));
  oai022aa1n02x5               g144(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n240));
  nona22aa1n02x5               g145(.a(new_n233), .b(new_n239), .c(new_n240), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n238), .c(new_n237), .d(new_n233), .o1(\s[22] ));
  inv000aa1d42x5               g147(.a(\a[22] ), .o1(new_n243));
  xroi22aa1d06x4               g148(.a(new_n236), .b(\b[20] ), .c(new_n243), .d(\b[21] ), .out0(new_n244));
  and003aa1n02x5               g149(.a(new_n207), .b(new_n244), .c(new_n224), .o(new_n245));
  oaoi03aa1n02x5               g150(.a(\a[22] ), .b(\b[21] ), .c(new_n237), .o1(new_n246));
  tech160nm_fiao0012aa1n02p5x5 g151(.a(new_n246), .b(new_n231), .c(new_n244), .o(new_n247));
  tech160nm_fixorc02aa1n04x5   g152(.a(\a[23] ), .b(\b[22] ), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n247), .c(new_n194), .d(new_n245), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n248), .b(new_n247), .c(new_n194), .d(new_n245), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n250), .out0(\s[23] ));
  norp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .out0(new_n254));
  and002aa1n02x5               g159(.a(\b[23] ), .b(\a[24] ), .o(new_n255));
  oai022aa1n02x5               g160(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n256));
  nona22aa1n02x5               g161(.a(new_n249), .b(new_n255), .c(new_n256), .out0(new_n257));
  aoai13aa1n03x5               g162(.a(new_n257), .b(new_n254), .c(new_n253), .d(new_n249), .o1(\s[24] ));
  and002aa1n02x5               g163(.a(new_n254), .b(new_n248), .o(new_n259));
  nano22aa1n02x4               g164(.a(new_n225), .b(new_n259), .c(new_n244), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n259), .b(new_n246), .c(new_n231), .d(new_n244), .o1(new_n261));
  aob012aa1n02x5               g166(.a(new_n256), .b(\b[23] ), .c(\a[24] ), .out0(new_n262));
  nanp02aa1n02x5               g167(.a(new_n261), .b(new_n262), .o1(new_n263));
  tech160nm_fixorc02aa1n05x5   g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n194), .d(new_n260), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n264), .b(new_n263), .c(new_n194), .d(new_n260), .o1(new_n266));
  norb02aa1n02x5               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  norp02aa1n02x5               g172(.a(\b[24] ), .b(\a[25] ), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  xorc02aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .out0(new_n270));
  and002aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .o(new_n271));
  oai022aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n272));
  nona22aa1n02x5               g177(.a(new_n265), .b(new_n271), .c(new_n272), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n270), .c(new_n269), .d(new_n265), .o1(\s[26] ));
  nanp02aa1n02x5               g179(.a(new_n270), .b(new_n264), .o1(new_n275));
  nona23aa1n03x5               g180(.a(new_n244), .b(new_n259), .c(new_n225), .d(new_n275), .out0(new_n276));
  aoi013aa1n03x5               g181(.a(new_n276), .b(new_n192), .c(new_n193), .d(new_n190), .o1(new_n277));
  nano23aa1d12x5               g182(.a(new_n225), .b(new_n275), .c(new_n259), .d(new_n244), .out0(new_n278));
  aob012aa1n02x5               g183(.a(new_n272), .b(\b[25] ), .c(\a[26] ), .out0(new_n279));
  aoai13aa1n09x5               g184(.a(new_n279), .b(new_n275), .c(new_n261), .d(new_n262), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n280), .c(new_n194), .d(new_n278), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n275), .b(new_n261), .c(new_n262), .o1(new_n283));
  nona22aa1n02x4               g188(.a(new_n279), .b(new_n283), .c(new_n281), .out0(new_n284));
  oa0012aa1n03x5               g189(.a(new_n282), .b(new_n284), .c(new_n277), .o(\s[27] ));
  inv000aa1d42x5               g190(.a(\a[27] ), .o1(new_n286));
  nanb02aa1n02x5               g191(.a(\b[26] ), .b(new_n286), .out0(new_n287));
  xorc02aa1n02x5               g192(.a(\a[28] ), .b(\b[27] ), .out0(new_n288));
  oai022aa1n02x5               g193(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n289));
  aoi012aa1n02x5               g194(.a(new_n289), .b(\a[28] ), .c(\b[27] ), .o1(new_n290));
  tech160nm_finand02aa1n03p5x5 g195(.a(new_n282), .b(new_n290), .o1(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n287), .d(new_n282), .o1(\s[28] ));
  xorc02aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .out0(new_n293));
  inv000aa1d42x5               g198(.a(\b[27] ), .o1(new_n294));
  xroi22aa1d04x5               g199(.a(new_n294), .b(\a[28] ), .c(new_n286), .d(\b[26] ), .out0(new_n295));
  aoai13aa1n04x5               g200(.a(new_n295), .b(new_n280), .c(new_n194), .d(new_n278), .o1(new_n296));
  oaib12aa1n09x5               g201(.a(new_n289), .b(new_n294), .c(\a[28] ), .out0(new_n297));
  nand03aa1n02x5               g202(.a(new_n296), .b(new_n297), .c(new_n293), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n297), .o1(new_n299));
  oaoi13aa1n03x5               g204(.a(new_n299), .b(new_n295), .c(new_n277), .d(new_n280), .o1(new_n300));
  oai012aa1n03x5               g205(.a(new_n298), .b(new_n300), .c(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g207(.a(new_n281), .b(new_n293), .c(new_n288), .o(new_n303));
  oaoi03aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .o1(new_n304));
  oaoi13aa1n02x7               g209(.a(new_n304), .b(new_n303), .c(new_n277), .d(new_n280), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n303), .b(new_n280), .c(new_n194), .d(new_n278), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n304), .o1(new_n308));
  nanp03aa1n03x5               g213(.a(new_n307), .b(new_n308), .c(new_n306), .o1(new_n309));
  oai012aa1n03x5               g214(.a(new_n309), .b(new_n305), .c(new_n306), .o1(\s[30] ));
  and003aa1n02x5               g215(.a(new_n295), .b(new_n306), .c(new_n293), .o(new_n311));
  oaoi03aa1n02x5               g216(.a(\a[30] ), .b(\b[29] ), .c(new_n308), .o1(new_n312));
  oaoi13aa1n02x7               g217(.a(new_n312), .b(new_n311), .c(new_n277), .d(new_n280), .o1(new_n313));
  xorc02aa1n02x5               g218(.a(\a[31] ), .b(\b[30] ), .out0(new_n314));
  aoai13aa1n04x5               g219(.a(new_n311), .b(new_n280), .c(new_n194), .d(new_n278), .o1(new_n315));
  nanb03aa1n03x5               g220(.a(new_n312), .b(new_n315), .c(new_n314), .out0(new_n316));
  oai012aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n314), .o1(\s[31] ));
  norp02aa1n02x5               g222(.a(new_n105), .b(new_n102), .o1(new_n318));
  oai012aa1n02x5               g223(.a(new_n100), .b(new_n104), .c(new_n103), .o1(new_n319));
  oaib12aa1n02x5               g224(.a(new_n319), .b(new_n101), .c(new_n99), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n320), .b(new_n318), .out0(\s[3] ));
  oai012aa1n02x5               g226(.a(new_n106), .b(new_n318), .c(new_n101), .o1(new_n322));
  nanp02aa1n02x5               g227(.a(new_n322), .b(new_n108), .o1(\s[4] ));
  and002aa1n02x5               g228(.a(\b[3] ), .b(\a[4] ), .o(new_n324));
  nanp02aa1n02x5               g229(.a(\b[4] ), .b(\a[5] ), .o1(new_n325));
  nona23aa1n02x4               g230(.a(new_n108), .b(new_n325), .c(new_n125), .d(new_n324), .out0(new_n326));
  inv000aa1d42x5               g231(.a(new_n125), .o1(new_n327));
  aboi22aa1n03x5               g232(.a(new_n324), .b(new_n108), .c(new_n325), .d(new_n327), .out0(new_n328));
  norb02aa1n02x5               g233(.a(new_n326), .b(new_n328), .out0(\s[5] ));
  norb02aa1n02x5               g234(.a(new_n124), .b(new_n111), .out0(new_n330));
  xnbna2aa1n03x5               g235(.a(new_n330), .b(new_n326), .c(new_n327), .out0(\s[6] ));
  nanp03aa1n02x5               g236(.a(new_n326), .b(new_n327), .c(new_n330), .o1(new_n332));
  nanp02aa1n02x5               g237(.a(\b[6] ), .b(\a[7] ), .o1(new_n333));
  nano22aa1n02x4               g238(.a(new_n119), .b(new_n124), .c(new_n333), .out0(new_n334));
  nanp02aa1n02x5               g239(.a(new_n332), .b(new_n124), .o1(new_n335));
  aoi022aa1n02x5               g240(.a(new_n335), .b(new_n110), .c(new_n332), .d(new_n334), .o1(\s[7] ));
  aoi022aa1n02x5               g241(.a(new_n332), .b(new_n334), .c(new_n122), .d(new_n121), .o1(new_n337));
  aob012aa1n02x5               g242(.a(new_n120), .b(new_n332), .c(new_n334), .out0(new_n338));
  aoai13aa1n02x5               g243(.a(new_n338), .b(new_n337), .c(new_n115), .d(new_n116), .o1(\s[8] ));
  aoi112aa1n02x5               g244(.a(new_n127), .b(new_n129), .c(new_n108), .d(new_n118), .o1(new_n340));
  norb02aa1n02x5               g245(.a(new_n130), .b(new_n340), .out0(\s[9] ));
endmodule


