// Benchmark "adder" written by ABC on Wed Jul 17 17:16:34 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n210, new_n211, new_n212, new_n213, new_n214,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n244, new_n245, new_n246,
    new_n247, new_n248, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n270,
    new_n271, new_n272, new_n273, new_n274, new_n275, new_n276, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n284, new_n285, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n296,
    new_n299, new_n300, new_n302, new_n304, new_n306;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nand42aa1n03x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nor022aa1n16x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor022aa1n04x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  oai012aa1n02x5               g005(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n101));
  oa0022aa1n02x5               g006(.a(\a[6] ), .b(\b[5] ), .c(\a[5] ), .d(\b[4] ), .o(new_n102));
  and002aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o(new_n103));
  nand42aa1n03x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n98), .b(new_n104), .c(new_n100), .d(new_n99), .out0(new_n105));
  oai013aa1n06x5               g010(.a(new_n101), .b(new_n105), .c(new_n102), .d(new_n103), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand22aa1n12x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nor042aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  tech160nm_fioai012aa1n04x5   g014(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n110));
  nor022aa1n06x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nand22aa1n04x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nona23aa1n12x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  aoi012aa1n02x5               g020(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n116));
  oai012aa1n06x5               g021(.a(new_n116), .b(new_n115), .c(new_n110), .o1(new_n117));
  tech160nm_fixorc02aa1n03p5x5 g022(.a(\a[6] ), .b(\b[5] ), .out0(new_n118));
  tech160nm_fixorc02aa1n03p5x5 g023(.a(\a[5] ), .b(\b[4] ), .out0(new_n119));
  nano22aa1n03x7               g024(.a(new_n105), .b(new_n118), .c(new_n119), .out0(new_n120));
  tech160nm_fixorc02aa1n02p5x5 g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  aoai13aa1n06x5               g026(.a(new_n121), .b(new_n106), .c(new_n120), .d(new_n117), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[10] ), .b(\b[9] ), .out0(new_n123));
  xnbna2aa1n03x5               g028(.a(new_n123), .b(new_n122), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g029(.a(new_n123), .o1(new_n125));
  oaoi03aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  aoai13aa1n04x5               g032(.a(new_n127), .b(new_n125), .c(new_n122), .d(new_n97), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nand42aa1d28x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nor022aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nor022aa1n03x5               g036(.a(\b[11] ), .b(\a[12] ), .o1(new_n132));
  nand42aa1n08x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  aoi112aa1n02x5               g039(.a(new_n134), .b(new_n131), .c(new_n128), .d(new_n130), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n134), .b(new_n131), .c(new_n128), .d(new_n130), .o1(new_n136));
  norb02aa1n03x4               g041(.a(new_n136), .b(new_n135), .out0(\s[12] ));
  nano23aa1n03x5               g042(.a(new_n132), .b(new_n131), .c(new_n133), .d(new_n130), .out0(new_n138));
  and003aa1n02x5               g043(.a(new_n138), .b(new_n123), .c(new_n121), .o(new_n139));
  aoai13aa1n06x5               g044(.a(new_n139), .b(new_n106), .c(new_n120), .d(new_n117), .o1(new_n140));
  tech160nm_fiao0012aa1n02p5x5 g045(.a(new_n132), .b(new_n131), .c(new_n133), .o(new_n141));
  aoi012aa1n06x5               g046(.a(new_n141), .b(new_n138), .c(new_n126), .o1(new_n142));
  nand42aa1d28x5               g047(.a(\b[12] ), .b(\a[13] ), .o1(new_n143));
  nor042aa1n04x5               g048(.a(\b[12] ), .b(\a[13] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n140), .c(new_n142), .out0(\s[13] ));
  inv000aa1d42x5               g051(.a(\a[13] ), .o1(new_n147));
  inv000aa1d42x5               g052(.a(\b[12] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n140), .b(new_n142), .o1(new_n149));
  oaoi03aa1n02x5               g054(.a(new_n147), .b(new_n148), .c(new_n149), .o1(new_n150));
  xnrb03aa1n02x5               g055(.a(new_n150), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nand42aa1d28x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nano23aa1d18x5               g058(.a(new_n152), .b(new_n144), .c(new_n153), .d(new_n143), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n153), .b(new_n152), .c(new_n147), .d(new_n148), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n140), .d(new_n142), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  tech160nm_finand02aa1n03p5x5 g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nor042aa1n02x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nor002aa1n02x5               g065(.a(\b[15] ), .b(\a[16] ), .o1(new_n161));
  tech160nm_finand02aa1n03p5x5 g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n164), .b(new_n160), .c(new_n157), .d(new_n159), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n164), .b(new_n160), .c(new_n157), .d(new_n159), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(\s[16] ));
  nano23aa1n02x5               g072(.a(new_n161), .b(new_n160), .c(new_n162), .d(new_n159), .out0(new_n168));
  nand02aa1d04x5               g073(.a(new_n168), .b(new_n154), .o1(new_n169));
  nano32aa1n03x7               g074(.a(new_n169), .b(new_n138), .c(new_n123), .d(new_n121), .out0(new_n170));
  aoai13aa1n12x5               g075(.a(new_n170), .b(new_n106), .c(new_n120), .d(new_n117), .o1(new_n171));
  nona23aa1n02x4               g076(.a(new_n159), .b(new_n162), .c(new_n161), .d(new_n160), .out0(new_n172));
  nanp02aa1n02x5               g077(.a(new_n160), .b(new_n162), .o1(new_n173));
  oai122aa1n06x5               g078(.a(new_n173), .b(new_n172), .c(new_n156), .d(\b[15] ), .e(\a[16] ), .o1(new_n174));
  oab012aa1n09x5               g079(.a(new_n174), .b(new_n142), .c(new_n169), .out0(new_n175));
  nanp02aa1n12x5               g080(.a(new_n171), .b(new_n175), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g082(.a(\a[18] ), .o1(new_n178));
  inv000aa1d42x5               g083(.a(\a[17] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  xroi22aa1d04x5               g087(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n179), .o1(new_n184));
  oaoi03aa1n02x5               g089(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n185));
  nand42aa1n03x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  nor042aa1n03x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n190));
  norb02aa1n02x7               g095(.a(new_n189), .b(new_n190), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nand42aa1n04x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  nona22aa1n02x5               g100(.a(new_n189), .b(new_n195), .c(new_n187), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n195), .o1(new_n197));
  oaoi13aa1n06x5               g102(.a(new_n197), .b(new_n189), .c(\a[19] ), .d(\b[18] ), .o1(new_n198));
  norb02aa1n03x4               g103(.a(new_n196), .b(new_n198), .out0(\s[20] ));
  nano23aa1n03x5               g104(.a(new_n193), .b(new_n187), .c(new_n194), .d(new_n186), .out0(new_n200));
  nanp02aa1n02x5               g105(.a(new_n183), .b(new_n200), .o1(new_n201));
  oai022aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  oaib12aa1n09x5               g107(.a(new_n202), .b(new_n178), .c(\b[17] ), .out0(new_n203));
  nona23aa1n09x5               g108(.a(new_n186), .b(new_n194), .c(new_n193), .d(new_n187), .out0(new_n204));
  aoi012aa1n06x5               g109(.a(new_n193), .b(new_n187), .c(new_n194), .o1(new_n205));
  oai012aa1d24x5               g110(.a(new_n205), .b(new_n204), .c(new_n203), .o1(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n04x5               g112(.a(new_n207), .b(new_n201), .c(new_n171), .d(new_n175), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g114(.a(\b[20] ), .b(\a[21] ), .o1(new_n210));
  xorc02aa1n02x5               g115(.a(\a[21] ), .b(\b[20] ), .out0(new_n211));
  xorc02aa1n02x5               g116(.a(\a[22] ), .b(\b[21] ), .out0(new_n212));
  aoi112aa1n02x5               g117(.a(new_n210), .b(new_n212), .c(new_n208), .d(new_n211), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n212), .b(new_n210), .c(new_n208), .d(new_n211), .o1(new_n214));
  norb02aa1n02x7               g119(.a(new_n214), .b(new_n213), .out0(\s[22] ));
  inv000aa1d42x5               g120(.a(\a[21] ), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\a[22] ), .o1(new_n217));
  xroi22aa1d04x5               g122(.a(new_n216), .b(\b[20] ), .c(new_n217), .d(\b[21] ), .out0(new_n218));
  nanp03aa1n02x5               g123(.a(new_n218), .b(new_n183), .c(new_n200), .o1(new_n219));
  inv000aa1d42x5               g124(.a(\b[21] ), .o1(new_n220));
  oaoi03aa1n12x5               g125(.a(new_n217), .b(new_n220), .c(new_n210), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoi012aa1n02x5               g127(.a(new_n222), .b(new_n206), .c(new_n218), .o1(new_n223));
  aoai13aa1n04x5               g128(.a(new_n223), .b(new_n219), .c(new_n171), .d(new_n175), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g130(.a(\b[22] ), .b(\a[23] ), .o1(new_n226));
  nanp02aa1n02x5               g131(.a(\b[22] ), .b(\a[23] ), .o1(new_n227));
  xorc02aa1n12x5               g132(.a(\a[24] ), .b(\b[23] ), .out0(new_n228));
  aoi112aa1n02x7               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoai13aa1n03x5               g134(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x7               g135(.a(new_n230), .b(new_n229), .out0(\s[24] ));
  nano22aa1n02x4               g136(.a(new_n226), .b(new_n228), .c(new_n227), .out0(new_n232));
  inv000aa1n02x5               g137(.a(new_n232), .o1(new_n233));
  nor002aa1n02x5               g138(.a(new_n219), .b(new_n233), .o1(new_n234));
  inv000aa1n03x5               g139(.a(new_n205), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n218), .b(new_n235), .c(new_n200), .d(new_n185), .o1(new_n236));
  orn002aa1n02x5               g141(.a(\a[23] ), .b(\b[22] ), .o(new_n237));
  oao003aa1n02x5               g142(.a(\a[24] ), .b(\b[23] ), .c(new_n237), .carry(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n233), .c(new_n236), .d(new_n221), .o1(new_n239));
  tech160nm_fixorc02aa1n05x5   g144(.a(\a[25] ), .b(\b[24] ), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n239), .c(new_n176), .d(new_n234), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(new_n240), .b(new_n239), .c(new_n176), .d(new_n234), .o1(new_n242));
  norb02aa1n02x7               g147(.a(new_n241), .b(new_n242), .out0(\s[25] ));
  nor042aa1n03x5               g148(.a(\b[24] ), .b(\a[25] ), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[26] ), .b(\b[25] ), .out0(new_n245));
  nona22aa1n02x5               g150(.a(new_n241), .b(new_n245), .c(new_n244), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n244), .o1(new_n247));
  aobi12aa1n06x5               g152(.a(new_n245), .b(new_n241), .c(new_n247), .out0(new_n248));
  norb02aa1n03x4               g153(.a(new_n246), .b(new_n248), .out0(\s[26] ));
  and002aa1n06x5               g154(.a(new_n245), .b(new_n240), .o(new_n250));
  nano22aa1n03x7               g155(.a(new_n219), .b(new_n232), .c(new_n250), .out0(new_n251));
  nand22aa1n09x5               g156(.a(new_n176), .b(new_n251), .o1(new_n252));
  oao003aa1n02x5               g157(.a(\a[26] ), .b(\b[25] ), .c(new_n247), .carry(new_n253));
  aobi12aa1n09x5               g158(.a(new_n253), .b(new_n239), .c(new_n250), .out0(new_n254));
  nor042aa1n03x5               g159(.a(\b[26] ), .b(\a[27] ), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(\b[26] ), .b(\a[27] ), .o1(new_n256));
  norb02aa1n02x5               g161(.a(new_n256), .b(new_n255), .out0(new_n257));
  xnbna2aa1n03x5               g162(.a(new_n257), .b(new_n254), .c(new_n252), .out0(\s[27] ));
  inv000aa1d42x5               g163(.a(new_n255), .o1(new_n259));
  xnrc02aa1n02x5               g164(.a(\b[27] ), .b(\a[28] ), .out0(new_n260));
  aobi12aa1n06x5               g165(.a(new_n256), .b(new_n254), .c(new_n252), .out0(new_n261));
  nano22aa1n03x5               g166(.a(new_n261), .b(new_n259), .c(new_n260), .out0(new_n262));
  aobi12aa1n06x5               g167(.a(new_n251), .b(new_n171), .c(new_n175), .out0(new_n263));
  aoai13aa1n06x5               g168(.a(new_n232), .b(new_n222), .c(new_n206), .d(new_n218), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n250), .o1(new_n265));
  aoai13aa1n06x5               g170(.a(new_n253), .b(new_n265), .c(new_n264), .d(new_n238), .o1(new_n266));
  oaih12aa1n02x5               g171(.a(new_n256), .b(new_n266), .c(new_n263), .o1(new_n267));
  tech160nm_fiaoi012aa1n02p5x5 g172(.a(new_n260), .b(new_n267), .c(new_n259), .o1(new_n268));
  norp02aa1n03x5               g173(.a(new_n268), .b(new_n262), .o1(\s[28] ));
  nano22aa1n02x4               g174(.a(new_n260), .b(new_n259), .c(new_n256), .out0(new_n270));
  oaih12aa1n02x5               g175(.a(new_n270), .b(new_n266), .c(new_n263), .o1(new_n271));
  oao003aa1n02x5               g176(.a(\a[28] ), .b(\b[27] ), .c(new_n259), .carry(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[28] ), .b(\a[29] ), .out0(new_n273));
  tech160nm_fiaoi012aa1n02p5x5 g178(.a(new_n273), .b(new_n271), .c(new_n272), .o1(new_n274));
  aobi12aa1n06x5               g179(.a(new_n270), .b(new_n254), .c(new_n252), .out0(new_n275));
  nano22aa1n03x5               g180(.a(new_n275), .b(new_n272), .c(new_n273), .out0(new_n276));
  norp02aa1n03x5               g181(.a(new_n274), .b(new_n276), .o1(\s[29] ));
  xorb03aa1n02x5               g182(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g183(.a(new_n257), .b(new_n273), .c(new_n260), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n279), .b(new_n266), .c(new_n263), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[29] ), .b(\b[28] ), .c(new_n272), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[29] ), .b(\a[30] ), .out0(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  aobi12aa1n06x5               g188(.a(new_n279), .b(new_n254), .c(new_n252), .out0(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n281), .c(new_n282), .out0(new_n285));
  norp02aa1n03x5               g190(.a(new_n283), .b(new_n285), .o1(\s[30] ));
  norb03aa1n02x5               g191(.a(new_n270), .b(new_n282), .c(new_n273), .out0(new_n287));
  aobi12aa1n06x5               g192(.a(new_n287), .b(new_n254), .c(new_n252), .out0(new_n288));
  oao003aa1n02x5               g193(.a(\a[30] ), .b(\b[29] ), .c(new_n281), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[30] ), .b(\a[31] ), .out0(new_n290));
  nano22aa1n03x5               g195(.a(new_n288), .b(new_n289), .c(new_n290), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n287), .b(new_n266), .c(new_n263), .o1(new_n292));
  tech160nm_fiaoi012aa1n05x5   g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  norp02aa1n03x5               g198(.a(new_n293), .b(new_n291), .o1(\s[31] ));
  xnrb03aa1n02x5               g199(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g200(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n296));
  xorb03aa1n02x5               g201(.a(new_n296), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g202(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai112aa1n02x5               g203(.a(new_n116), .b(new_n119), .c(new_n115), .d(new_n110), .o1(new_n299));
  aob012aa1n02x5               g204(.a(new_n299), .b(\b[4] ), .c(\a[5] ), .out0(new_n300));
  xnrc02aa1n02x5               g205(.a(new_n300), .b(new_n118), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g206(.a(new_n103), .b(new_n300), .c(new_n118), .o(new_n302));
  xnrb03aa1n02x5               g207(.a(new_n302), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g208(.a(\a[7] ), .b(\b[6] ), .c(new_n302), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g210(.a(new_n106), .b(new_n121), .c(new_n120), .d(new_n117), .o1(new_n306));
  norb02aa1n02x5               g211(.a(new_n122), .b(new_n306), .out0(\s[9] ));
endmodule


