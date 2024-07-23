// Benchmark "adder" written by ABC on Thu Jul 18 15:25:36 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  norp02aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor022aa1n08x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  aoi012aa1n02x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand22aa1n06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oai012aa1n02x7               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n107));
  oai012aa1n09x5               g012(.a(new_n101), .b(new_n107), .c(new_n105), .o1(new_n108));
  nand42aa1n03x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor022aa1n04x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  norp02aa1n03x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nona23aa1n03x5               g017(.a(new_n109), .b(new_n112), .c(new_n111), .d(new_n110), .out0(new_n113));
  tech160nm_fixorc02aa1n02p5x5 g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .out0(new_n115));
  norb03aa1n03x5               g020(.a(new_n114), .b(new_n113), .c(new_n115), .out0(new_n116));
  nanp02aa1n03x5               g021(.a(new_n116), .b(new_n108), .o1(new_n117));
  xorc02aa1n03x5               g022(.a(\a[7] ), .b(\b[6] ), .out0(new_n118));
  nand42aa1n02x5               g023(.a(new_n118), .b(new_n109), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n114), .b(new_n111), .c(new_n110), .o1(new_n120));
  norp02aa1n02x5               g025(.a(new_n120), .b(new_n119), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[8] ), .o1(new_n122));
  norp02aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .o1(new_n123));
  aob012aa1n02x5               g028(.a(new_n123), .b(\b[7] ), .c(\a[8] ), .out0(new_n124));
  oaib12aa1n02x5               g029(.a(new_n124), .b(\b[7] ), .c(new_n122), .out0(new_n125));
  nona22aa1n02x4               g030(.a(new_n117), .b(new_n121), .c(new_n125), .out0(new_n126));
  nanp02aa1n02x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n97), .b(new_n126), .c(new_n127), .o1(new_n128));
  xnrb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  oabi12aa1n06x5               g034(.a(new_n125), .b(new_n120), .c(new_n119), .out0(new_n130));
  nor042aa1n03x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand02aa1n06x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nano23aa1n06x5               g037(.a(new_n131), .b(new_n97), .c(new_n127), .d(new_n132), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n130), .c(new_n108), .d(new_n116), .o1(new_n134));
  aoi012aa1d24x5               g039(.a(new_n131), .b(new_n97), .c(new_n132), .o1(new_n135));
  nor002aa1n16x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nanp02aa1n04x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  inv040aa1n02x5               g044(.a(new_n136), .o1(new_n140));
  inv040aa1n02x5               g045(.a(new_n135), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n138), .b(new_n141), .c(new_n126), .d(new_n133), .o1(new_n142));
  nor042aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nanp02aa1n04x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n140), .out0(\s[12] ));
  nona23aa1n09x5               g051(.a(new_n144), .b(new_n137), .c(new_n136), .d(new_n143), .out0(new_n147));
  norb02aa1n02x5               g052(.a(new_n133), .b(new_n147), .out0(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n130), .c(new_n108), .d(new_n116), .o1(new_n149));
  oaoi03aa1n09x5               g054(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n150));
  oabi12aa1n18x5               g055(.a(new_n150), .b(new_n147), .c(new_n135), .out0(new_n151));
  inv000aa1d42x5               g056(.a(new_n151), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(new_n149), .b(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n08x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1d28x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n04x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nand42aa1n20x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1d15x5               g065(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  aoi012aa1n09x5               g067(.a(new_n159), .b(new_n155), .c(new_n160), .o1(new_n163));
  aoai13aa1n04x5               g068(.a(new_n163), .b(new_n162), .c(new_n149), .d(new_n152), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nand42aa1d28x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor042aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand42aa1n08x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoai13aa1n04x5               g076(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n166), .b(new_n171), .c(new_n164), .d(new_n167), .o1(new_n173));
  norb02aa1n03x4               g078(.a(new_n172), .b(new_n173), .out0(\s[16] ));
  nano23aa1n03x7               g079(.a(new_n136), .b(new_n143), .c(new_n144), .d(new_n137), .out0(new_n175));
  nano23aa1n03x7               g080(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n176));
  inv020aa1n03x5               g081(.a(new_n176), .o1(new_n177));
  nano32aa1n03x7               g082(.a(new_n177), .b(new_n161), .c(new_n175), .d(new_n133), .out0(new_n178));
  aoai13aa1n12x5               g083(.a(new_n178), .b(new_n130), .c(new_n108), .d(new_n116), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n163), .o1(new_n180));
  aoai13aa1n04x5               g085(.a(new_n176), .b(new_n180), .c(new_n151), .d(new_n161), .o1(new_n181));
  oai012aa1n02x5               g086(.a(new_n169), .b(new_n168), .c(new_n166), .o1(new_n182));
  nand23aa1d12x5               g087(.a(new_n179), .b(new_n181), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g089(.a(\b[16] ), .b(\a[17] ), .o1(new_n185));
  nand02aa1n04x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  tech160nm_fiaoi012aa1n05x5   g091(.a(new_n185), .b(new_n183), .c(new_n186), .o1(new_n187));
  xnrb03aa1n03x5               g092(.a(new_n187), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  aoai13aa1n06x5               g093(.a(new_n161), .b(new_n150), .c(new_n175), .d(new_n141), .o1(new_n189));
  nand22aa1n03x5               g094(.a(new_n189), .b(new_n163), .o1(new_n190));
  aobi12aa1n06x5               g095(.a(new_n182), .b(new_n190), .c(new_n176), .out0(new_n191));
  nor042aa1n06x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nand02aa1d08x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  nano23aa1d15x5               g098(.a(new_n185), .b(new_n192), .c(new_n193), .d(new_n186), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  aoi012aa1n12x5               g100(.a(new_n192), .b(new_n185), .c(new_n193), .o1(new_n196));
  aoai13aa1n04x5               g101(.a(new_n196), .b(new_n195), .c(new_n191), .d(new_n179), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  inv000aa1n06x5               g105(.a(new_n200), .o1(new_n201));
  inv020aa1n03x5               g106(.a(new_n196), .o1(new_n202));
  nand22aa1n03x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n200), .out0(new_n204));
  aoai13aa1n03x5               g109(.a(new_n204), .b(new_n202), .c(new_n183), .d(new_n194), .o1(new_n205));
  nor042aa1n03x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n04x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  aobi12aa1n02x7               g113(.a(new_n208), .b(new_n205), .c(new_n201), .out0(new_n209));
  aoi112aa1n02x5               g114(.a(new_n200), .b(new_n208), .c(new_n197), .d(new_n204), .o1(new_n210));
  nor002aa1n02x5               g115(.a(new_n209), .b(new_n210), .o1(\s[20] ));
  nano23aa1n06x5               g116(.a(new_n200), .b(new_n206), .c(new_n207), .d(new_n203), .out0(new_n212));
  nand02aa1d06x5               g117(.a(new_n212), .b(new_n194), .o1(new_n213));
  nona23aa1n09x5               g118(.a(new_n207), .b(new_n203), .c(new_n200), .d(new_n206), .out0(new_n214));
  oaoi03aa1n12x5               g119(.a(\a[20] ), .b(\b[19] ), .c(new_n201), .o1(new_n215));
  inv040aa1n03x5               g120(.a(new_n215), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n214), .c(new_n196), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n04x5               g123(.a(new_n218), .b(new_n213), .c(new_n191), .d(new_n179), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  inv040aa1n03x5               g126(.a(new_n221), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n213), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[21] ), .b(\b[20] ), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n224), .b(new_n217), .c(new_n183), .d(new_n223), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .out0(new_n226));
  aobi12aa1n02x7               g131(.a(new_n226), .b(new_n225), .c(new_n222), .out0(new_n227));
  aoi112aa1n02x5               g132(.a(new_n221), .b(new_n226), .c(new_n219), .d(new_n224), .o1(new_n228));
  nor002aa1n02x5               g133(.a(new_n227), .b(new_n228), .o1(\s[22] ));
  nano22aa1n03x7               g134(.a(new_n213), .b(new_n224), .c(new_n226), .out0(new_n230));
  inv030aa1n02x5               g135(.a(new_n230), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\a[21] ), .o1(new_n232));
  inv020aa1n04x5               g137(.a(\a[22] ), .o1(new_n233));
  xroi22aa1d06x4               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  oaoi03aa1n09x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .o1(new_n235));
  aoi012aa1d18x5               g140(.a(new_n235), .b(new_n217), .c(new_n234), .o1(new_n236));
  aoai13aa1n04x5               g141(.a(new_n236), .b(new_n231), .c(new_n191), .d(new_n179), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n236), .o1(new_n241));
  xorc02aa1n12x5               g146(.a(\a[23] ), .b(\b[22] ), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n241), .c(new_n183), .d(new_n230), .o1(new_n243));
  tech160nm_fixorc02aa1n05x5   g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aobi12aa1n06x5               g149(.a(new_n244), .b(new_n243), .c(new_n240), .out0(new_n245));
  aoi112aa1n03x4               g150(.a(new_n239), .b(new_n244), .c(new_n237), .d(new_n242), .o1(new_n246));
  nor042aa1n03x5               g151(.a(new_n245), .b(new_n246), .o1(\s[24] ));
  and002aa1n02x5               g152(.a(new_n244), .b(new_n242), .o(new_n248));
  inv000aa1n03x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n234), .c(new_n212), .d(new_n194), .out0(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n234), .b(new_n215), .c(new_n212), .d(new_n202), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n235), .o1(new_n253));
  nanp02aa1n02x5               g158(.a(\b[23] ), .b(\a[24] ), .o1(new_n254));
  oai022aa1n02x5               g159(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n255), .b(new_n254), .o1(new_n256));
  aoai13aa1n12x5               g161(.a(new_n256), .b(new_n249), .c(new_n252), .d(new_n253), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  aoai13aa1n04x5               g163(.a(new_n258), .b(new_n251), .c(new_n191), .d(new_n179), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  tech160nm_fixorc02aa1n05x5   g167(.a(\a[25] ), .b(\b[24] ), .out0(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n257), .c(new_n183), .d(new_n250), .o1(new_n264));
  tech160nm_fixorc02aa1n05x5   g169(.a(\a[26] ), .b(\b[25] ), .out0(new_n265));
  aobi12aa1n02x7               g170(.a(new_n265), .b(new_n264), .c(new_n262), .out0(new_n266));
  aoi112aa1n02x5               g171(.a(new_n261), .b(new_n265), .c(new_n259), .d(new_n263), .o1(new_n267));
  nor002aa1n02x5               g172(.a(new_n266), .b(new_n267), .o1(\s[26] ));
  aoai13aa1n02x5               g173(.a(new_n182), .b(new_n177), .c(new_n189), .d(new_n163), .o1(new_n269));
  and002aa1n02x5               g174(.a(new_n265), .b(new_n263), .o(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  nano23aa1d12x5               g176(.a(new_n271), .b(new_n213), .c(new_n248), .d(new_n234), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n269), .c(new_n126), .d(new_n178), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n274));
  aobi12aa1n12x5               g179(.a(new_n274), .b(new_n257), .c(new_n270), .out0(new_n275));
  xorc02aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n273), .c(new_n275), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  inv040aa1n03x5               g183(.a(new_n278), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n248), .b(new_n235), .c(new_n217), .d(new_n234), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n274), .b(new_n271), .c(new_n280), .d(new_n256), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n276), .b(new_n281), .c(new_n183), .d(new_n272), .o1(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n283), .b(new_n282), .c(new_n279), .o1(new_n284));
  aobi12aa1n02x7               g189(.a(new_n276), .b(new_n273), .c(new_n275), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n285), .b(new_n279), .c(new_n283), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  norb02aa1n02x5               g193(.a(new_n276), .b(new_n283), .out0(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n281), .c(new_n183), .d(new_n272), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n288), .b(new_n290), .c(new_n291), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n289), .b(new_n273), .c(new_n275), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  norb03aa1n02x5               g202(.a(new_n276), .b(new_n288), .c(new_n283), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n298), .b(new_n281), .c(new_n183), .d(new_n272), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[29] ), .b(\b[28] ), .c(new_n291), .carry(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n297), .b(new_n299), .c(new_n300), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n298), .b(new_n273), .c(new_n275), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n297), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  norb02aa1n02x5               g210(.a(new_n298), .b(new_n297), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n281), .c(new_n183), .d(new_n272), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n300), .carry(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n305), .b(new_n307), .c(new_n308), .o1(new_n309));
  aobi12aa1n02x7               g214(.a(new_n306), .b(new_n273), .c(new_n275), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n305), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n03x5               g221(.a(new_n112), .b(new_n108), .c(new_n111), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb03aa1n02x5               g223(.a(new_n110), .b(new_n317), .c(new_n109), .out0(new_n319));
  xnbna2aa1n03x5               g224(.a(new_n115), .b(new_n319), .c(new_n109), .out0(\s[7] ));
  aoib12aa1n03x5               g225(.a(new_n123), .b(new_n319), .c(new_n119), .out0(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(new_n122), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


