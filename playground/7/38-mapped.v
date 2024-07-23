// Benchmark "adder" written by ABC on Wed Jul 17 15:51:43 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n275, new_n276, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n326, new_n329, new_n332, new_n334,
    new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n12x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi012aa1n06x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor022aa1n08x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  aoi012aa1n12x5               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n108));
  oai012aa1n12x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nor022aa1n08x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n112), .b(new_n111), .c(new_n113), .d(new_n110), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[5] ), .b(\a[6] ), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .out0(new_n116));
  nor043aa1n04x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\a[5] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[4] ), .o1(new_n119));
  nand42aa1n03x5               g024(.a(new_n119), .b(new_n118), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  tech160nm_fiaoi012aa1n03p5x5 g026(.a(new_n110), .b(new_n113), .c(new_n111), .o1(new_n122));
  oaib12aa1n06x5               g027(.a(new_n122), .b(new_n114), .c(new_n121), .out0(new_n123));
  nand42aa1n03x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nanb02aa1n02x5               g029(.a(new_n97), .b(new_n124), .out0(new_n125));
  inv000aa1n02x5               g030(.a(new_n125), .o1(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n127));
  nor002aa1n16x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  and002aa1n02x7               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  norp02aa1n02x5               g034(.a(new_n129), .b(new_n128), .o1(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  nor002aa1d32x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand02aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n06x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n128), .o1(new_n135));
  aoai13aa1n04x5               g040(.a(new_n135), .b(new_n129), .c(new_n127), .d(new_n98), .o1(new_n136));
  oai022aa1n06x5               g041(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n137));
  inv030aa1n03x5               g042(.a(new_n137), .o1(new_n138));
  tech160nm_fiao0012aa1n02p5x5 g043(.a(new_n129), .b(new_n127), .c(new_n138), .o(new_n139));
  mtn022aa1n02x5               g044(.a(new_n136), .b(new_n139), .sa(new_n134), .o1(\s[11] ));
  inv040aa1d32x5               g045(.a(\a[12] ), .o1(new_n141));
  inv040aa1d28x5               g046(.a(\b[11] ), .o1(new_n142));
  nand42aa1n20x5               g047(.a(new_n142), .b(new_n141), .o1(new_n143));
  nand42aa1n16x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1d28x5               g049(.a(new_n143), .b(new_n144), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoi112aa1n02x5               g051(.a(new_n146), .b(new_n132), .c(new_n136), .d(new_n134), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n146), .b(new_n132), .c(new_n136), .d(new_n133), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(\s[12] ));
  norb03aa1n12x5               g054(.a(new_n124), .b(new_n97), .c(new_n128), .out0(new_n150));
  nona23aa1d18x5               g055(.a(new_n134), .b(new_n150), .c(new_n145), .d(new_n129), .out0(new_n151));
  inv020aa1n02x5               g056(.a(new_n151), .o1(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n153));
  inv000aa1d42x5               g058(.a(\a[10] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(\b[9] ), .o1(new_n155));
  inv000aa1n04x5               g060(.a(new_n132), .o1(new_n156));
  oai112aa1n06x5               g061(.a(new_n156), .b(new_n133), .c(new_n155), .d(new_n154), .o1(new_n157));
  tech160nm_fioaoi03aa1n03p5x5 g062(.a(new_n141), .b(new_n142), .c(new_n132), .o1(new_n158));
  oai013aa1d12x5               g063(.a(new_n158), .b(new_n157), .c(new_n145), .d(new_n138), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  norp02aa1n24x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  xobna2aa1n03x5               g068(.a(new_n163), .b(new_n153), .c(new_n160), .out0(\s[13] ));
  inv000aa1d42x5               g069(.a(new_n161), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n163), .c(new_n153), .d(new_n160), .o1(new_n166));
  xorb03aa1n02x5               g071(.a(new_n166), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand42aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nona23aa1n03x5               g074(.a(new_n169), .b(new_n162), .c(new_n161), .d(new_n168), .out0(new_n170));
  oaih12aa1n02x5               g075(.a(new_n169), .b(new_n168), .c(new_n161), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n170), .c(new_n153), .d(new_n160), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n12x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  nand42aa1n03x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nanb02aa1n06x5               g081(.a(new_n174), .b(new_n176), .out0(new_n177));
  nanb02aa1n02x5               g082(.a(new_n177), .b(new_n172), .out0(new_n178));
  nor042aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nand42aa1n03x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  nanp03aa1n02x5               g086(.a(new_n178), .b(new_n175), .c(new_n181), .o1(new_n182));
  tech160nm_fiaoi012aa1n04x5   g087(.a(new_n181), .b(new_n178), .c(new_n175), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n182), .b(new_n183), .out0(\s[16] ));
  nano23aa1n03x7               g089(.a(new_n161), .b(new_n168), .c(new_n169), .d(new_n162), .out0(new_n185));
  nano23aa1n02x5               g090(.a(new_n174), .b(new_n179), .c(new_n180), .d(new_n176), .out0(new_n186));
  nano22aa1n06x5               g091(.a(new_n151), .b(new_n185), .c(new_n186), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n188));
  nor003aa1n03x5               g093(.a(new_n170), .b(new_n177), .c(new_n181), .o1(new_n189));
  norp03aa1n04x5               g094(.a(new_n171), .b(new_n177), .c(new_n181), .o1(new_n190));
  oaoi03aa1n02x5               g095(.a(\a[16] ), .b(\b[15] ), .c(new_n175), .o1(new_n191));
  aoi112aa1n09x5               g096(.a(new_n191), .b(new_n190), .c(new_n159), .d(new_n189), .o1(new_n192));
  xorc02aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n188), .c(new_n192), .out0(\s[17] ));
  inv030aa1d32x5               g099(.a(\a[17] ), .o1(new_n195));
  inv000aa1d48x5               g100(.a(\b[16] ), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(new_n196), .b(new_n195), .o1(new_n197));
  nano23aa1n02x5               g102(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n198));
  nanb02aa1n02x5               g103(.a(new_n102), .b(new_n198), .out0(new_n199));
  nano23aa1n03x7               g104(.a(new_n113), .b(new_n110), .c(new_n111), .d(new_n112), .out0(new_n200));
  nona22aa1n02x4               g105(.a(new_n200), .b(new_n115), .c(new_n116), .out0(new_n201));
  aobi12aa1n06x5               g106(.a(new_n122), .b(new_n200), .c(new_n121), .out0(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n201), .c(new_n199), .d(new_n108), .o1(new_n203));
  nano32aa1n02x4               g108(.a(new_n157), .b(new_n137), .c(new_n143), .d(new_n144), .out0(new_n204));
  oaib12aa1n06x5               g109(.a(new_n189), .b(new_n204), .c(new_n158), .out0(new_n205));
  nona22aa1n06x5               g110(.a(new_n205), .b(new_n190), .c(new_n191), .out0(new_n206));
  aoai13aa1n02x5               g111(.a(new_n193), .b(new_n206), .c(new_n203), .d(new_n187), .o1(new_n207));
  nor002aa1d32x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  nand42aa1d28x5               g113(.a(\b[17] ), .b(\a[18] ), .o1(new_n209));
  nanb02aa1d24x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  xobna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n197), .out0(\s[18] ));
  nanp02aa1n02x5               g116(.a(\b[16] ), .b(\a[17] ), .o1(new_n212));
  nano22aa1d15x5               g117(.a(new_n210), .b(new_n197), .c(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n12x5               g119(.a(new_n209), .b(new_n208), .c(new_n195), .d(new_n196), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n215), .b(new_n214), .c(new_n188), .d(new_n192), .o1(new_n216));
  xorb03aa1n02x5               g121(.a(new_n216), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand02aa1d08x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nor042aa1n06x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand22aa1n09x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  aoi112aa1n03x5               g128(.a(new_n219), .b(new_n223), .c(new_n216), .d(new_n220), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n219), .c(new_n216), .d(new_n220), .o1(new_n225));
  norb02aa1n03x4               g130(.a(new_n225), .b(new_n224), .out0(\s[20] ));
  nano23aa1n09x5               g131(.a(new_n219), .b(new_n221), .c(new_n222), .d(new_n220), .out0(new_n227));
  nanp02aa1n02x5               g132(.a(new_n213), .b(new_n227), .o1(new_n228));
  nona23aa1n09x5               g133(.a(new_n222), .b(new_n220), .c(new_n219), .d(new_n221), .out0(new_n229));
  aoi012aa1n06x5               g134(.a(new_n221), .b(new_n219), .c(new_n222), .o1(new_n230));
  oai012aa1n12x5               g135(.a(new_n230), .b(new_n229), .c(new_n215), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n228), .c(new_n188), .d(new_n192), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g139(.a(\b[20] ), .b(\a[21] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[21] ), .b(\b[20] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[22] ), .b(\b[21] ), .out0(new_n237));
  aoi112aa1n03x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoai13aa1n03x5               g143(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n03x4               g144(.a(new_n239), .b(new_n238), .out0(\s[22] ));
  inv000aa1d42x5               g145(.a(\a[21] ), .o1(new_n241));
  inv040aa1d32x5               g146(.a(\a[22] ), .o1(new_n242));
  xroi22aa1d06x4               g147(.a(new_n241), .b(\b[20] ), .c(new_n242), .d(\b[21] ), .out0(new_n243));
  nanp03aa1n02x5               g148(.a(new_n243), .b(new_n213), .c(new_n227), .o1(new_n244));
  inv000aa1d42x5               g149(.a(\b[21] ), .o1(new_n245));
  oao003aa1n02x5               g150(.a(new_n242), .b(new_n245), .c(new_n235), .carry(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n231), .c(new_n243), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n244), .c(new_n188), .d(new_n192), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  xorc02aa1n12x5               g155(.a(\a[23] ), .b(\b[22] ), .out0(new_n251));
  xorc02aa1n12x5               g156(.a(\a[24] ), .b(\b[23] ), .out0(new_n252));
  aoi112aa1n03x5               g157(.a(new_n250), .b(new_n252), .c(new_n248), .d(new_n251), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n252), .b(new_n250), .c(new_n248), .d(new_n251), .o1(new_n254));
  norb02aa1n03x4               g159(.a(new_n254), .b(new_n253), .out0(\s[24] ));
  nand02aa1d06x5               g160(.a(new_n252), .b(new_n251), .o1(new_n256));
  norb03aa1n03x5               g161(.a(new_n243), .b(new_n228), .c(new_n256), .out0(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n206), .c(new_n203), .d(new_n187), .o1(new_n258));
  inv020aa1n02x5               g163(.a(new_n215), .o1(new_n259));
  inv020aa1n03x5               g164(.a(new_n230), .o1(new_n260));
  aoai13aa1n06x5               g165(.a(new_n243), .b(new_n260), .c(new_n227), .d(new_n259), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n246), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n263));
  oab012aa1n04x5               g168(.a(new_n263), .b(\a[24] ), .c(\b[23] ), .out0(new_n264));
  aoai13aa1n12x5               g169(.a(new_n264), .b(new_n256), .c(new_n261), .d(new_n262), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  xnrc02aa1n12x5               g171(.a(\b[24] ), .b(\a[25] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n268), .b(new_n258), .c(new_n266), .out0(\s[25] ));
  nor042aa1n03x5               g174(.a(\b[24] ), .b(\a[25] ), .o1(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  nanp02aa1n06x5               g176(.a(new_n188), .b(new_n192), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n268), .b(new_n265), .c(new_n272), .d(new_n257), .o1(new_n273));
  tech160nm_fixnrc02aa1n05x5   g178(.a(\b[25] ), .b(\a[26] ), .out0(new_n274));
  nanp03aa1n03x5               g179(.a(new_n273), .b(new_n271), .c(new_n274), .o1(new_n275));
  aoi012aa1n06x5               g180(.a(new_n274), .b(new_n273), .c(new_n271), .o1(new_n276));
  norb02aa1n03x4               g181(.a(new_n275), .b(new_n276), .out0(\s[26] ));
  norp02aa1n12x5               g182(.a(new_n274), .b(new_n267), .o1(new_n278));
  nano23aa1n06x5               g183(.a(new_n228), .b(new_n256), .c(new_n243), .d(new_n278), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n206), .c(new_n203), .d(new_n187), .o1(new_n280));
  nand02aa1d10x5               g185(.a(new_n265), .b(new_n278), .o1(new_n281));
  oao003aa1n02x5               g186(.a(\a[26] ), .b(\b[25] ), .c(new_n271), .carry(new_n282));
  xorc02aa1n12x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  inv000aa1d42x5               g188(.a(new_n283), .o1(new_n284));
  aoi013aa1n06x4               g189(.a(new_n284), .b(new_n280), .c(new_n281), .d(new_n282), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n256), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n246), .c(new_n231), .d(new_n243), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n278), .o1(new_n288));
  aoai13aa1n06x5               g193(.a(new_n282), .b(new_n288), .c(new_n287), .d(new_n264), .o1(new_n289));
  aoi112aa1n02x5               g194(.a(new_n289), .b(new_n283), .c(new_n272), .d(new_n279), .o1(new_n290));
  norp02aa1n02x5               g195(.a(new_n285), .b(new_n290), .o1(\s[27] ));
  nor042aa1n03x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  xnrc02aa1n12x5               g198(.a(\b[27] ), .b(\a[28] ), .out0(new_n294));
  nano22aa1n03x5               g199(.a(new_n285), .b(new_n293), .c(new_n294), .out0(new_n295));
  aobi12aa1n06x5               g200(.a(new_n279), .b(new_n188), .c(new_n192), .out0(new_n296));
  oaih12aa1n02x5               g201(.a(new_n283), .b(new_n289), .c(new_n296), .o1(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n294), .b(new_n297), .c(new_n293), .o1(new_n298));
  norp02aa1n03x5               g203(.a(new_n298), .b(new_n295), .o1(\s[28] ));
  norb02aa1n02x5               g204(.a(new_n283), .b(new_n294), .out0(new_n300));
  inv000aa1n02x5               g205(.a(new_n300), .o1(new_n301));
  aoi013aa1n03x5               g206(.a(new_n301), .b(new_n280), .c(new_n281), .d(new_n282), .o1(new_n302));
  oao003aa1n02x5               g207(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[28] ), .b(\a[29] ), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n302), .b(new_n303), .c(new_n304), .out0(new_n305));
  oaih12aa1n02x5               g210(.a(new_n300), .b(new_n289), .c(new_n296), .o1(new_n306));
  aoi012aa1n03x5               g211(.a(new_n304), .b(new_n306), .c(new_n303), .o1(new_n307));
  norp02aa1n03x5               g212(.a(new_n307), .b(new_n305), .o1(\s[29] ));
  xorb03aa1n02x5               g213(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n12x5               g214(.a(new_n283), .b(new_n304), .c(new_n294), .out0(new_n310));
  oaih12aa1n02x5               g215(.a(new_n310), .b(new_n289), .c(new_n296), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .c(new_n303), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[29] ), .b(\a[30] ), .out0(new_n313));
  tech160nm_fiaoi012aa1n02p5x5 g218(.a(new_n313), .b(new_n311), .c(new_n312), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n310), .o1(new_n315));
  aoi013aa1n03x5               g220(.a(new_n315), .b(new_n280), .c(new_n281), .d(new_n282), .o1(new_n316));
  nano22aa1n03x5               g221(.a(new_n316), .b(new_n312), .c(new_n313), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n314), .b(new_n317), .o1(\s[30] ));
  norb02aa1n02x5               g223(.a(new_n310), .b(new_n313), .out0(new_n319));
  inv000aa1n02x5               g224(.a(new_n319), .o1(new_n320));
  aoi013aa1n03x5               g225(.a(new_n320), .b(new_n280), .c(new_n281), .d(new_n282), .o1(new_n321));
  oao003aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .c(new_n312), .carry(new_n322));
  xnrc02aa1n02x5               g227(.a(\b[30] ), .b(\a[31] ), .out0(new_n323));
  nano22aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n323), .out0(new_n324));
  oaih12aa1n02x5               g229(.a(new_n319), .b(new_n289), .c(new_n296), .o1(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n323), .b(new_n325), .c(new_n322), .o1(new_n326));
  norp02aa1n03x5               g231(.a(new_n326), .b(new_n324), .o1(\s[31] ));
  xnrb03aa1n02x5               g232(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g233(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g235(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g236(.a(new_n118), .b(new_n119), .c(new_n109), .o1(new_n332));
  xnrb03aa1n02x5               g237(.a(new_n332), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n03x5               g238(.a(\a[6] ), .b(\b[5] ), .c(new_n332), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g240(.a(new_n113), .b(new_n334), .c(new_n112), .o1(new_n336));
  xnrb03aa1n03x5               g241(.a(new_n336), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g242(.a(new_n203), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule

