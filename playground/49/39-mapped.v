// Benchmark "adder" written by ABC on Thu Jul 18 14:55:09 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n322, new_n323, new_n324, new_n326, new_n327,
    new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n09x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nor002aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand42aa1d28x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nano23aa1n03x5               g009(.a(new_n101), .b(new_n103), .c(new_n104), .d(new_n102), .out0(new_n105));
  tech160nm_fiao0012aa1n02p5x5 g010(.a(new_n101), .b(new_n103), .c(new_n102), .o(new_n106));
  tech160nm_fiaoi012aa1n05x5   g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand42aa1n20x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor002aa1d32x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n20x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nano23aa1n02x4               g016(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  nand23aa1n03x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  nano22aa1n02x4               g022(.a(new_n110), .b(new_n109), .c(new_n111), .out0(new_n118));
  oai022aa1n02x5               g023(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  aoai13aa1n09x5               g024(.a(new_n116), .b(new_n119), .c(new_n118), .d(new_n117), .o1(new_n120));
  oai012aa1n09x5               g025(.a(new_n120), .b(new_n107), .c(new_n115), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  nanb02aa1n02x5               g027(.a(new_n97), .b(new_n122), .out0(new_n123));
  inv000aa1d42x5               g028(.a(new_n123), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(new_n121), .b(new_n124), .o1(new_n125));
  nor002aa1n03x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nand02aa1n03x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  norp02aa1n02x5               g034(.a(new_n126), .b(new_n97), .o1(new_n130));
  nand42aa1n04x5               g035(.a(new_n125), .b(new_n130), .o1(new_n131));
  nand02aa1n03x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor002aa1n12x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nano22aa1n02x4               g038(.a(new_n133), .b(new_n127), .c(new_n132), .out0(new_n134));
  nanb02aa1n02x5               g039(.a(new_n133), .b(new_n132), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n131), .b(new_n127), .o1(new_n136));
  aoi022aa1n02x5               g041(.a(new_n136), .b(new_n135), .c(new_n131), .d(new_n134), .o1(\s[11] ));
  nor002aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1d04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n140), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n141));
  aoi112aa1n02x5               g046(.a(new_n133), .b(new_n140), .c(new_n131), .d(new_n134), .o1(new_n142));
  norb02aa1n02x7               g047(.a(new_n141), .b(new_n142), .out0(\s[12] ));
  nona23aa1n09x5               g048(.a(new_n132), .b(new_n139), .c(new_n138), .d(new_n133), .out0(new_n144));
  nona23aa1n02x4               g049(.a(new_n127), .b(new_n122), .c(new_n97), .d(new_n126), .out0(new_n145));
  nor042aa1n02x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  oai012aa1n02x5               g051(.a(new_n127), .b(new_n126), .c(new_n97), .o1(new_n147));
  ao0012aa1n03x5               g052(.a(new_n138), .b(new_n133), .c(new_n139), .o(new_n148));
  oabi12aa1n03x5               g053(.a(new_n148), .b(new_n144), .c(new_n147), .out0(new_n149));
  tech160nm_fiao0012aa1n03p5x5 g054(.a(new_n149), .b(new_n121), .c(new_n146), .o(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n04x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand22aa1n03x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  nor002aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoi112aa1n02x5               g063(.a(new_n152), .b(new_n158), .c(new_n150), .d(new_n153), .o1(new_n159));
  oab012aa1n02x5               g064(.a(new_n159), .b(new_n154), .c(new_n157), .out0(\s[14] ));
  nona23aa1n03x5               g065(.a(new_n156), .b(new_n153), .c(new_n152), .d(new_n155), .out0(new_n161));
  inv000aa1n03x5               g066(.a(new_n161), .o1(new_n162));
  aoai13aa1n03x5               g067(.a(new_n162), .b(new_n149), .c(new_n121), .d(new_n146), .o1(new_n163));
  oai012aa1n02x5               g068(.a(new_n156), .b(new_n155), .c(new_n152), .o1(new_n164));
  nor022aa1n16x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand42aa1n03x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n163), .c(new_n164), .out0(\s[15] ));
  aob012aa1n03x5               g073(.a(new_n167), .b(new_n163), .c(new_n164), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n165), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n167), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n171), .c(new_n163), .d(new_n164), .o1(new_n172));
  nor022aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand42aa1n03x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  aoib12aa1n02x5               g080(.a(new_n165), .b(new_n174), .c(new_n173), .out0(new_n176));
  aoi022aa1n03x5               g081(.a(new_n172), .b(new_n175), .c(new_n169), .d(new_n176), .o1(\s[16] ));
  nona23aa1n09x5               g082(.a(new_n174), .b(new_n166), .c(new_n165), .d(new_n173), .out0(new_n178));
  nona22aa1n03x5               g083(.a(new_n146), .b(new_n161), .c(new_n178), .out0(new_n179));
  nanb02aa1n06x5               g084(.a(new_n179), .b(new_n121), .out0(new_n180));
  aobi12aa1n02x7               g085(.a(new_n164), .b(new_n149), .c(new_n162), .out0(new_n181));
  aoi012aa1n02x5               g086(.a(new_n173), .b(new_n165), .c(new_n174), .o1(new_n182));
  oai112aa1n06x5               g087(.a(new_n180), .b(new_n182), .c(new_n181), .d(new_n178), .o1(new_n183));
  xorb03aa1n03x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[17] ), .o1(new_n185));
  nanb02aa1n12x5               g090(.a(\b[16] ), .b(new_n185), .out0(new_n186));
  oaoi13aa1n09x5               g091(.a(new_n179), .b(new_n120), .c(new_n107), .d(new_n115), .o1(new_n187));
  nano23aa1n06x5               g092(.a(new_n138), .b(new_n133), .c(new_n139), .d(new_n132), .out0(new_n188));
  oaoi03aa1n02x5               g093(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n189));
  aoai13aa1n04x5               g094(.a(new_n162), .b(new_n148), .c(new_n188), .d(new_n189), .o1(new_n190));
  aoai13aa1n06x5               g095(.a(new_n182), .b(new_n178), .c(new_n190), .d(new_n164), .o1(new_n191));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  oaih12aa1n02x5               g097(.a(new_n192), .b(new_n191), .c(new_n187), .o1(new_n193));
  tech160nm_fixorc02aa1n04x5   g098(.a(\a[18] ), .b(\b[17] ), .out0(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n194), .b(new_n193), .c(new_n186), .out0(\s[18] ));
  inv000aa1d42x5               g100(.a(\a[18] ), .o1(new_n196));
  xroi22aa1d04x5               g101(.a(new_n185), .b(\b[16] ), .c(new_n196), .d(\b[17] ), .out0(new_n197));
  oaih12aa1n02x5               g102(.a(new_n197), .b(new_n191), .c(new_n187), .o1(new_n198));
  oai022aa1n04x5               g103(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n199));
  oaib12aa1n09x5               g104(.a(new_n199), .b(new_n196), .c(\b[17] ), .out0(new_n200));
  nor002aa1d32x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nand02aa1n06x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nanb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n198), .c(new_n200), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g111(.a(\a[18] ), .b(\b[17] ), .c(new_n186), .o1(new_n207));
  aoai13aa1n03x5               g112(.a(new_n204), .b(new_n207), .c(new_n183), .d(new_n197), .o1(new_n208));
  inv040aa1n03x5               g113(.a(new_n201), .o1(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n203), .c(new_n198), .d(new_n200), .o1(new_n210));
  nor002aa1n12x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand42aa1n08x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  norb02aa1n02x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  aoib12aa1n02x5               g118(.a(new_n201), .b(new_n212), .c(new_n211), .out0(new_n214));
  aoi022aa1n03x5               g119(.a(new_n210), .b(new_n213), .c(new_n208), .d(new_n214), .o1(\s[20] ));
  nano23aa1n09x5               g120(.a(new_n201), .b(new_n211), .c(new_n212), .d(new_n202), .out0(new_n216));
  nand23aa1n04x5               g121(.a(new_n216), .b(new_n192), .c(new_n194), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  oaih12aa1n02x5               g123(.a(new_n218), .b(new_n191), .c(new_n187), .o1(new_n219));
  nona23aa1n09x5               g124(.a(new_n212), .b(new_n202), .c(new_n201), .d(new_n211), .out0(new_n220));
  oaoi03aa1n06x5               g125(.a(\a[20] ), .b(\b[19] ), .c(new_n209), .o1(new_n221));
  oabi12aa1n18x5               g126(.a(new_n221), .b(new_n220), .c(new_n200), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n219), .c(new_n223), .out0(\s[21] ));
  aoai13aa1n06x5               g130(.a(new_n224), .b(new_n222), .c(new_n183), .d(new_n218), .o1(new_n226));
  nor042aa1n03x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  inv040aa1n02x5               g132(.a(new_n227), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n224), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n228), .b(new_n229), .c(new_n219), .d(new_n223), .o1(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  norp02aa1n02x5               g136(.a(new_n231), .b(new_n227), .o1(new_n232));
  aoi022aa1n03x5               g137(.a(new_n230), .b(new_n231), .c(new_n226), .d(new_n232), .o1(\s[22] ));
  nanp02aa1n02x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  xnrc02aa1n02x5               g139(.a(\b[21] ), .b(\a[22] ), .out0(new_n235));
  nano22aa1n03x7               g140(.a(new_n235), .b(new_n228), .c(new_n234), .out0(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n217), .out0(new_n237));
  oai012aa1n06x5               g142(.a(new_n237), .b(new_n191), .c(new_n187), .o1(new_n238));
  oao003aa1n02x5               g143(.a(\a[22] ), .b(\b[21] ), .c(new_n228), .carry(new_n239));
  inv000aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  aoi012aa1n02x5               g145(.a(new_n240), .b(new_n222), .c(new_n236), .o1(new_n241));
  tech160nm_finand02aa1n05x5   g146(.a(new_n238), .b(new_n241), .o1(new_n242));
  xorc02aa1n12x5               g147(.a(\a[23] ), .b(\b[22] ), .out0(new_n243));
  aoi112aa1n02x5               g148(.a(new_n243), .b(new_n240), .c(new_n222), .d(new_n236), .o1(new_n244));
  aoi022aa1n02x5               g149(.a(new_n242), .b(new_n243), .c(new_n238), .d(new_n244), .o1(\s[23] ));
  nanp02aa1n03x5               g150(.a(new_n242), .b(new_n243), .o1(new_n246));
  nor042aa1n06x5               g151(.a(\b[22] ), .b(\a[23] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n243), .o1(new_n249));
  aoai13aa1n02x7               g154(.a(new_n248), .b(new_n249), .c(new_n238), .d(new_n241), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .out0(new_n251));
  norp02aa1n02x5               g156(.a(new_n251), .b(new_n247), .o1(new_n252));
  aoi022aa1n02x7               g157(.a(new_n250), .b(new_n251), .c(new_n246), .d(new_n252), .o1(\s[24] ));
  nano32aa1n02x4               g158(.a(new_n217), .b(new_n251), .c(new_n236), .d(new_n243), .out0(new_n254));
  oaih12aa1n02x5               g159(.a(new_n254), .b(new_n191), .c(new_n187), .o1(new_n255));
  aoai13aa1n06x5               g160(.a(new_n236), .b(new_n221), .c(new_n216), .d(new_n207), .o1(new_n256));
  and002aa1n03x5               g161(.a(new_n251), .b(new_n243), .o(new_n257));
  inv000aa1n06x5               g162(.a(new_n257), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .carry(new_n259));
  aoai13aa1n12x5               g164(.a(new_n259), .b(new_n258), .c(new_n256), .d(new_n239), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n255), .c(new_n261), .out0(\s[25] ));
  aoai13aa1n03x5               g168(.a(new_n262), .b(new_n260), .c(new_n183), .d(new_n254), .o1(new_n264));
  nor042aa1n03x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n262), .o1(new_n267));
  aoai13aa1n02x5               g172(.a(new_n266), .b(new_n267), .c(new_n255), .d(new_n261), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[26] ), .b(\b[25] ), .out0(new_n269));
  norp02aa1n02x5               g174(.a(new_n269), .b(new_n265), .o1(new_n270));
  aoi022aa1n03x5               g175(.a(new_n268), .b(new_n269), .c(new_n264), .d(new_n270), .o1(\s[26] ));
  and002aa1n03x5               g176(.a(new_n269), .b(new_n262), .o(new_n272));
  inv000aa1n02x5               g177(.a(new_n272), .o1(new_n273));
  nano23aa1n06x5               g178(.a(new_n273), .b(new_n217), .c(new_n257), .d(new_n236), .out0(new_n274));
  oai012aa1n06x5               g179(.a(new_n274), .b(new_n191), .c(new_n187), .o1(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n276));
  aobi12aa1n09x5               g181(.a(new_n276), .b(new_n260), .c(new_n272), .out0(new_n277));
  xorc02aa1n12x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n277), .c(new_n275), .out0(\s[27] ));
  aoai13aa1n03x5               g184(.a(new_n257), .b(new_n240), .c(new_n222), .d(new_n236), .o1(new_n280));
  aoai13aa1n02x7               g185(.a(new_n276), .b(new_n273), .c(new_n280), .d(new_n259), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n278), .b(new_n281), .c(new_n183), .d(new_n274), .o1(new_n282));
  norp02aa1n02x5               g187(.a(\b[26] ), .b(\a[27] ), .o1(new_n283));
  inv000aa1n03x5               g188(.a(new_n283), .o1(new_n284));
  inv000aa1n02x5               g189(.a(new_n278), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n284), .b(new_n285), .c(new_n277), .d(new_n275), .o1(new_n286));
  tech160nm_fixorc02aa1n03p5x5 g191(.a(\a[28] ), .b(\b[27] ), .out0(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n283), .o1(new_n288));
  aoi022aa1n03x5               g193(.a(new_n286), .b(new_n287), .c(new_n282), .d(new_n288), .o1(\s[28] ));
  and002aa1n02x5               g194(.a(new_n287), .b(new_n278), .o(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n281), .c(new_n183), .d(new_n274), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n290), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n284), .carry(new_n293));
  aoai13aa1n02x7               g198(.a(new_n293), .b(new_n292), .c(new_n277), .d(new_n275), .o1(new_n294));
  xorc02aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n293), .b(new_n295), .out0(new_n296));
  aoi022aa1n03x5               g201(.a(new_n294), .b(new_n295), .c(new_n291), .d(new_n296), .o1(\s[29] ));
  xnrb03aa1n02x5               g202(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n09x5               g203(.a(new_n285), .b(new_n287), .c(new_n295), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n281), .c(new_n183), .d(new_n274), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n299), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n301), .c(new_n277), .d(new_n275), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .out0(new_n304));
  norb02aa1n02x5               g209(.a(new_n302), .b(new_n304), .out0(new_n305));
  aoi022aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n300), .d(new_n305), .o1(\s[30] ));
  xorc02aa1n02x5               g211(.a(\a[31] ), .b(\b[30] ), .out0(new_n307));
  nano32aa1n06x5               g212(.a(new_n285), .b(new_n304), .c(new_n287), .d(new_n295), .out0(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n281), .c(new_n183), .d(new_n274), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n310), .c(new_n277), .d(new_n275), .o1(new_n312));
  and002aa1n02x5               g217(.a(\b[29] ), .b(\a[30] ), .o(new_n313));
  oabi12aa1n02x5               g218(.a(new_n307), .b(\a[30] ), .c(\b[29] ), .out0(new_n314));
  oab012aa1n02x4               g219(.a(new_n314), .b(new_n302), .c(new_n313), .out0(new_n315));
  aoi022aa1n03x5               g220(.a(new_n312), .b(new_n307), .c(new_n309), .d(new_n315), .o1(\s[31] ));
  xorb03aa1n02x5               g221(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g222(.a(new_n104), .b(new_n100), .c(new_n103), .o1(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnrc02aa1n02x5               g224(.a(new_n107), .b(new_n113), .out0(\s[5] ));
  norb02aa1n02x5               g225(.a(new_n109), .b(new_n108), .out0(new_n321));
  orn002aa1n02x5               g226(.a(\a[5] ), .b(\b[4] ), .o(new_n322));
  aoai13aa1n02x5               g227(.a(new_n113), .b(new_n106), .c(new_n105), .d(new_n100), .o1(new_n323));
  nanb03aa1n02x5               g228(.a(new_n117), .b(new_n323), .c(new_n109), .out0(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n321), .c(new_n323), .d(new_n322), .o1(\s[6] ));
  inv000aa1d42x5               g230(.a(new_n110), .o1(new_n326));
  aoi022aa1n02x5               g231(.a(new_n324), .b(new_n109), .c(new_n326), .d(new_n111), .o1(new_n327));
  nanp02aa1n02x5               g232(.a(new_n324), .b(new_n118), .o1(new_n328));
  norb02aa1n02x5               g233(.a(new_n328), .b(new_n327), .out0(\s[7] ));
  xnbna2aa1n03x5               g234(.a(new_n114), .b(new_n328), .c(new_n326), .out0(\s[8] ));
  xorb03aa1n02x5               g235(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

