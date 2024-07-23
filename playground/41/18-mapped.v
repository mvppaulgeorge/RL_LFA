// Benchmark "adder" written by ABC on Thu Jul 18 09:06:50 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n315, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  and002aa1n02x5               g002(.a(\b[9] ), .b(\a[10] ), .o(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nor042aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand02aa1n03x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nanb02aa1n06x5               g006(.a(new_n100), .b(new_n101), .out0(new_n102));
  tech160nm_fixorc02aa1n05x5   g007(.a(\a[7] ), .b(\b[6] ), .out0(new_n103));
  norb02aa1n02x7               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[5] ), .o1(new_n105));
  inv040aa1d30x5               g010(.a(\a[6] ), .o1(new_n106));
  xroi22aa1d04x5               g011(.a(new_n105), .b(\b[4] ), .c(new_n106), .d(\b[5] ), .out0(new_n107));
  norp02aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  norp02aa1n03x5               g014(.a(\b[2] ), .b(\a[3] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nona23aa1n06x5               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nand02aa1n02x5               g019(.a(\b[0] ), .b(\a[1] ), .o1(new_n115));
  tech160nm_fioai012aa1n04x5   g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  oai012aa1n02x5               g021(.a(new_n109), .b(new_n110), .c(new_n108), .o1(new_n117));
  oai012aa1n09x5               g022(.a(new_n117), .b(new_n112), .c(new_n116), .o1(new_n118));
  nand23aa1n06x5               g023(.a(new_n118), .b(new_n104), .c(new_n107), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  nor042aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  aob012aa1n03x5               g026(.a(new_n121), .b(\b[5] ), .c(\a[6] ), .out0(new_n122));
  oaib12aa1n06x5               g027(.a(new_n122), .b(\b[5] ), .c(new_n106), .out0(new_n123));
  aoi113aa1n09x5               g028(.a(new_n120), .b(new_n100), .c(new_n123), .d(new_n103), .e(new_n101), .o1(new_n124));
  nanp02aa1n09x5               g029(.a(new_n119), .b(new_n124), .o1(new_n125));
  xnrc02aa1n12x5               g030(.a(\b[8] ), .b(\a[9] ), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  nanp02aa1n03x5               g032(.a(new_n125), .b(new_n127), .o1(new_n128));
  nona32aa1n03x5               g033(.a(new_n128), .b(new_n99), .c(new_n98), .d(new_n97), .out0(new_n129));
  xnrc02aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  aoai13aa1n02x5               g035(.a(new_n130), .b(new_n99), .c(new_n125), .d(new_n127), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(new_n129), .b(new_n131), .o1(\s[10] ));
  xnrc02aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .out0(new_n133));
  nona22aa1n06x5               g038(.a(new_n129), .b(new_n133), .c(new_n98), .out0(new_n134));
  xorc02aa1n02x5               g039(.a(\a[11] ), .b(\b[10] ), .out0(new_n135));
  aoib12aa1n02x5               g040(.a(new_n135), .b(new_n129), .c(new_n98), .out0(new_n136));
  norb02aa1n02x5               g041(.a(new_n134), .b(new_n136), .out0(\s[11] ));
  nor042aa1n06x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norp02aa1n12x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nanp02aa1n04x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  nona22aa1n02x4               g046(.a(new_n134), .b(new_n141), .c(new_n138), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n138), .o1(new_n143));
  nanb02aa1n06x5               g048(.a(new_n139), .b(new_n140), .out0(new_n144));
  tech160nm_fiaoi012aa1n02p5x5 g049(.a(new_n144), .b(new_n134), .c(new_n143), .o1(new_n145));
  norb02aa1n03x4               g050(.a(new_n142), .b(new_n145), .out0(\s[12] ));
  nona23aa1n06x5               g051(.a(new_n127), .b(new_n135), .c(new_n130), .d(new_n144), .out0(new_n147));
  aoi112aa1n03x5               g052(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n148));
  norp02aa1n02x5               g053(.a(new_n148), .b(new_n97), .o1(new_n149));
  nor043aa1n02x5               g054(.a(new_n149), .b(new_n133), .c(new_n144), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n139), .b(new_n138), .c(new_n140), .o1(new_n151));
  norb02aa1n03x5               g056(.a(new_n151), .b(new_n150), .out0(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n147), .c(new_n119), .d(new_n124), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand22aa1n03x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n12x5               g063(.a(\a[15] ), .b(\b[14] ), .out0(new_n159));
  nor042aa1n03x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand22aa1n04x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n06x5               g066(.a(new_n155), .b(new_n160), .c(new_n161), .d(new_n156), .out0(new_n162));
  aoi012aa1n09x5               g067(.a(new_n160), .b(new_n155), .c(new_n161), .o1(new_n163));
  inv000aa1n02x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n159), .b(new_n164), .c(new_n153), .d(new_n162), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n159), .b(new_n164), .c(new_n153), .d(new_n162), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  xnrc02aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .out0(new_n168));
  oai112aa1n03x5               g073(.a(new_n165), .b(new_n168), .c(\b[14] ), .d(\a[15] ), .o1(new_n169));
  oaoi13aa1n06x5               g074(.a(new_n168), .b(new_n165), .c(\a[15] ), .d(\b[14] ), .o1(new_n170));
  norb02aa1n03x4               g075(.a(new_n169), .b(new_n170), .out0(\s[16] ));
  xorc02aa1n03x5               g076(.a(\a[16] ), .b(\b[15] ), .out0(new_n172));
  nanp03aa1n09x5               g077(.a(new_n162), .b(new_n159), .c(new_n172), .o1(new_n173));
  nor042aa1n04x5               g078(.a(new_n147), .b(new_n173), .o1(new_n174));
  inv020aa1n03x5               g079(.a(new_n174), .o1(new_n175));
  aoi112aa1n03x5               g080(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(new_n176), .o1(new_n177));
  oai112aa1n02x5               g082(.a(new_n135), .b(new_n141), .c(new_n97), .d(new_n148), .o1(new_n178));
  aoi012aa1n02x5               g083(.a(new_n173), .b(new_n178), .c(new_n151), .o1(new_n179));
  norp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  aoi013aa1n03x5               g085(.a(new_n180), .b(new_n164), .c(new_n159), .d(new_n172), .o1(new_n181));
  nano22aa1n03x7               g086(.a(new_n179), .b(new_n181), .c(new_n177), .out0(new_n182));
  aoai13aa1n12x5               g087(.a(new_n182), .b(new_n175), .c(new_n119), .d(new_n124), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g089(.a(\a[18] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n03x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n185), .out0(\s[18] ));
  oai112aa1n04x5               g094(.a(new_n177), .b(new_n181), .c(new_n152), .d(new_n173), .o1(new_n190));
  xroi22aa1d06x4               g095(.a(new_n186), .b(\b[16] ), .c(new_n185), .d(\b[17] ), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n190), .c(new_n125), .d(new_n174), .o1(new_n192));
  norp02aa1n02x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  nand22aa1n06x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  aoi013aa1n06x4               g099(.a(new_n193), .b(new_n194), .c(new_n186), .d(new_n187), .o1(new_n195));
  nor002aa1d32x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanp02aa1n06x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n192), .c(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n06x5               g105(.a(new_n196), .o1(new_n201));
  aobi12aa1n06x5               g106(.a(new_n198), .b(new_n192), .c(new_n195), .out0(new_n202));
  norp02aa1n06x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanp02aa1n06x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nanb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(new_n205));
  nano22aa1n03x7               g110(.a(new_n202), .b(new_n201), .c(new_n205), .out0(new_n206));
  nona22aa1n02x4               g111(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(new_n207));
  oaib12aa1n06x5               g112(.a(new_n207), .b(\b[17] ), .c(new_n185), .out0(new_n208));
  aoai13aa1n02x5               g113(.a(new_n198), .b(new_n208), .c(new_n183), .d(new_n191), .o1(new_n209));
  tech160nm_fiaoi012aa1n02p5x5 g114(.a(new_n205), .b(new_n209), .c(new_n201), .o1(new_n210));
  norp02aa1n02x5               g115(.a(new_n210), .b(new_n206), .o1(\s[20] ));
  nona23aa1n09x5               g116(.a(new_n204), .b(new_n197), .c(new_n196), .d(new_n203), .out0(new_n212));
  norb02aa1n02x5               g117(.a(new_n191), .b(new_n212), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n190), .c(new_n125), .d(new_n174), .o1(new_n214));
  oaoi03aa1n09x5               g119(.a(\a[20] ), .b(\b[19] ), .c(new_n201), .o1(new_n215));
  inv030aa1n02x5               g120(.a(new_n215), .o1(new_n216));
  oai012aa1n18x5               g121(.a(new_n216), .b(new_n212), .c(new_n195), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  xorc02aa1n02x5               g123(.a(\a[21] ), .b(\b[20] ), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n214), .c(new_n218), .out0(\s[21] ));
  orn002aa1n24x5               g125(.a(\a[21] ), .b(\b[20] ), .o(new_n221));
  aobi12aa1n02x5               g126(.a(new_n219), .b(new_n214), .c(new_n218), .out0(new_n222));
  tech160nm_fixnrc02aa1n05x5   g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  nano22aa1n02x4               g128(.a(new_n222), .b(new_n221), .c(new_n223), .out0(new_n224));
  aoai13aa1n03x5               g129(.a(new_n219), .b(new_n217), .c(new_n183), .d(new_n213), .o1(new_n225));
  tech160nm_fiaoi012aa1n02p5x5 g130(.a(new_n223), .b(new_n225), .c(new_n221), .o1(new_n226));
  norp02aa1n02x5               g131(.a(new_n226), .b(new_n224), .o1(\s[22] ));
  nano23aa1n09x5               g132(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n228));
  nanp02aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  nano22aa1n03x7               g134(.a(new_n223), .b(new_n221), .c(new_n229), .out0(new_n230));
  and003aa1n03x7               g135(.a(new_n191), .b(new_n230), .c(new_n228), .o(new_n231));
  aoai13aa1n06x5               g136(.a(new_n231), .b(new_n190), .c(new_n125), .d(new_n174), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n230), .b(new_n215), .c(new_n228), .d(new_n208), .o1(new_n233));
  oaoi03aa1n12x5               g138(.a(\a[22] ), .b(\b[21] ), .c(new_n221), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  nand02aa1n03x5               g140(.a(new_n233), .b(new_n235), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n232), .c(new_n237), .out0(\s[23] ));
  nor042aa1n06x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aobi12aa1n06x5               g146(.a(new_n238), .b(new_n232), .c(new_n237), .out0(new_n242));
  xnrc02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .out0(new_n243));
  nano22aa1n03x7               g148(.a(new_n242), .b(new_n241), .c(new_n243), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n238), .b(new_n236), .c(new_n183), .d(new_n231), .o1(new_n245));
  tech160nm_fiaoi012aa1n02p5x5 g150(.a(new_n243), .b(new_n245), .c(new_n241), .o1(new_n246));
  norp02aa1n02x5               g151(.a(new_n246), .b(new_n244), .o1(\s[24] ));
  norb02aa1n02x5               g152(.a(new_n238), .b(new_n243), .out0(new_n248));
  inv000aa1n03x5               g153(.a(new_n248), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n191), .c(new_n230), .d(new_n228), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n190), .c(new_n125), .d(new_n174), .o1(new_n251));
  oao003aa1n02x5               g156(.a(\a[24] ), .b(\b[23] ), .c(new_n241), .carry(new_n252));
  aoai13aa1n12x5               g157(.a(new_n252), .b(new_n249), .c(new_n233), .d(new_n235), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xorc02aa1n12x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xnbna2aa1n03x5               g160(.a(new_n255), .b(new_n251), .c(new_n254), .out0(\s[25] ));
  nor042aa1n03x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  aobi12aa1n06x5               g163(.a(new_n255), .b(new_n251), .c(new_n254), .out0(new_n259));
  tech160nm_fixnrc02aa1n04x5   g164(.a(\b[25] ), .b(\a[26] ), .out0(new_n260));
  nano22aa1n03x7               g165(.a(new_n259), .b(new_n258), .c(new_n260), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n255), .b(new_n253), .c(new_n183), .d(new_n250), .o1(new_n262));
  aoi012aa1n02x7               g167(.a(new_n260), .b(new_n262), .c(new_n258), .o1(new_n263));
  norp02aa1n02x5               g168(.a(new_n263), .b(new_n261), .o1(\s[26] ));
  norb02aa1n12x5               g169(.a(new_n255), .b(new_n260), .out0(new_n265));
  and003aa1n06x5               g170(.a(new_n231), .b(new_n265), .c(new_n248), .o(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n190), .c(new_n125), .d(new_n174), .o1(new_n267));
  oao003aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .c(new_n258), .carry(new_n268));
  aobi12aa1n12x5               g173(.a(new_n268), .b(new_n253), .c(new_n265), .out0(new_n269));
  xorc02aa1n02x5               g174(.a(\a[27] ), .b(\b[26] ), .out0(new_n270));
  xnbna2aa1n03x5               g175(.a(new_n270), .b(new_n267), .c(new_n269), .out0(\s[27] ));
  norp02aa1n02x5               g176(.a(\b[26] ), .b(\a[27] ), .o1(new_n272));
  inv040aa1n03x5               g177(.a(new_n272), .o1(new_n273));
  aobi12aa1n02x7               g178(.a(new_n270), .b(new_n267), .c(new_n269), .out0(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[27] ), .b(\a[28] ), .out0(new_n275));
  nano22aa1n03x5               g180(.a(new_n274), .b(new_n273), .c(new_n275), .out0(new_n276));
  aoai13aa1n03x5               g181(.a(new_n248), .b(new_n234), .c(new_n217), .d(new_n230), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n265), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n268), .b(new_n278), .c(new_n277), .d(new_n252), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n270), .b(new_n279), .c(new_n183), .d(new_n266), .o1(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n275), .b(new_n280), .c(new_n273), .o1(new_n281));
  norp02aa1n03x5               g186(.a(new_n281), .b(new_n276), .o1(\s[28] ));
  norb02aa1n02x5               g187(.a(new_n270), .b(new_n275), .out0(new_n283));
  aobi12aa1n03x5               g188(.a(new_n283), .b(new_n267), .c(new_n269), .out0(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n273), .carry(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nano22aa1n03x5               g191(.a(new_n284), .b(new_n285), .c(new_n286), .out0(new_n287));
  aoai13aa1n02x5               g192(.a(new_n283), .b(new_n279), .c(new_n183), .d(new_n266), .o1(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n286), .b(new_n288), .c(new_n285), .o1(new_n289));
  norp02aa1n03x5               g194(.a(new_n289), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n115), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g196(.a(new_n270), .b(new_n286), .c(new_n275), .out0(new_n292));
  aobi12aa1n06x5               g197(.a(new_n292), .b(new_n267), .c(new_n269), .out0(new_n293));
  oao003aa1n02x5               g198(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[29] ), .b(\a[30] ), .out0(new_n295));
  nano22aa1n03x7               g200(.a(new_n293), .b(new_n294), .c(new_n295), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n292), .b(new_n279), .c(new_n183), .d(new_n266), .o1(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n295), .b(new_n297), .c(new_n294), .o1(new_n298));
  norp02aa1n03x5               g203(.a(new_n298), .b(new_n296), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n292), .b(new_n295), .out0(new_n300));
  aobi12aa1n03x5               g205(.a(new_n300), .b(new_n267), .c(new_n269), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n300), .b(new_n279), .c(new_n183), .d(new_n266), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n116), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n116), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n118), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g216(.a(\b[4] ), .o1(new_n312));
  oaoi03aa1n02x5               g217(.a(new_n105), .b(new_n312), .c(new_n118), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(new_n106), .out0(\s[6] ));
  aoai13aa1n02x5               g219(.a(new_n103), .b(new_n123), .c(new_n118), .d(new_n107), .o1(new_n315));
  aoi112aa1n02x5               g220(.a(new_n123), .b(new_n103), .c(new_n118), .d(new_n107), .o1(new_n316));
  norb02aa1n02x5               g221(.a(new_n315), .b(new_n316), .out0(\s[7] ));
  orn002aa1n02x5               g222(.a(\a[7] ), .b(\b[6] ), .o(new_n318));
  xobna2aa1n03x5               g223(.a(new_n102), .b(new_n315), .c(new_n318), .out0(\s[8] ));
  xnbna2aa1n03x5               g224(.a(new_n127), .b(new_n119), .c(new_n124), .out0(\s[9] ));
endmodule


