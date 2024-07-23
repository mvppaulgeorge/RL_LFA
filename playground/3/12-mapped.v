// Benchmark "adder" written by ABC on Wed Jul 17 13:32:03 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n147, new_n148, new_n149,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n157, new_n158,
    new_n159, new_n160, new_n161, new_n163, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n207,
    new_n208, new_n209, new_n210, new_n211, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n222, new_n223,
    new_n224, new_n225, new_n226, new_n228, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n245, new_n246,
    new_n247, new_n248, new_n249, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n272, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n298, new_n301, new_n302, new_n304, new_n306, new_n308;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  inv020aa1n02x5               g003(.a(new_n98), .o1(new_n99));
  nor002aa1d32x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand42aa1n04x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor022aa1n12x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1d18x5               g008(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n104));
  xnrc02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .out0(new_n105));
  xorc02aa1n02x5               g010(.a(\a[5] ), .b(\b[4] ), .out0(new_n106));
  norb03aa1n09x5               g011(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n107));
  and002aa1n02x5               g012(.a(\b[3] ), .b(\a[4] ), .o(new_n108));
  nor022aa1n16x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  oab012aa1n02x4               g014(.a(new_n109), .b(\a[4] ), .c(\b[3] ), .out0(new_n110));
  nand42aa1n03x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nanb02aa1n12x5               g016(.a(new_n109), .b(new_n111), .out0(new_n112));
  nand42aa1n03x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  nor042aa1d18x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nand42aa1n16x5               g019(.a(\b[0] ), .b(\a[1] ), .o1(new_n115));
  oai012aa1n12x5               g020(.a(new_n113), .b(new_n114), .c(new_n115), .o1(new_n116));
  oaoi13aa1n12x5               g021(.a(new_n108), .b(new_n110), .c(new_n116), .d(new_n112), .o1(new_n117));
  inv000aa1d42x5               g022(.a(new_n100), .o1(new_n118));
  nand42aa1n03x5               g023(.a(new_n102), .b(new_n101), .o1(new_n119));
  aoi112aa1n03x5               g024(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  oab012aa1n04x5               g025(.a(new_n120), .b(\a[6] ), .c(\b[5] ), .out0(new_n121));
  oai112aa1n06x5               g026(.a(new_n118), .b(new_n119), .c(new_n104), .d(new_n121), .o1(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n122), .c(new_n117), .d(new_n107), .o1(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n97), .b(new_n124), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g030(.a(new_n97), .o1(new_n126));
  aoi112aa1n09x5               g031(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n127));
  oab012aa1n09x5               g032(.a(new_n127), .b(\a[10] ), .c(\b[9] ), .out0(new_n128));
  aoai13aa1n03x5               g033(.a(new_n128), .b(new_n126), .c(new_n124), .d(new_n99), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n12x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand02aa1n08x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nor022aa1n08x5               g037(.a(\b[11] ), .b(\a[12] ), .o1(new_n133));
  nand22aa1n09x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  aoi112aa1n02x5               g040(.a(new_n135), .b(new_n131), .c(new_n129), .d(new_n132), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n135), .b(new_n131), .c(new_n129), .d(new_n132), .o1(new_n137));
  norb02aa1n02x7               g042(.a(new_n137), .b(new_n136), .out0(\s[12] ));
  nona23aa1n09x5               g043(.a(new_n134), .b(new_n132), .c(new_n131), .d(new_n133), .out0(new_n139));
  nano22aa1n02x4               g044(.a(new_n139), .b(new_n97), .c(new_n123), .out0(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n122), .c(new_n117), .d(new_n107), .o1(new_n141));
  aoi012aa1n06x5               g046(.a(new_n133), .b(new_n131), .c(new_n134), .o1(new_n142));
  oai012aa1d24x5               g047(.a(new_n142), .b(new_n139), .c(new_n128), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n141), .b(new_n144), .o1(new_n145));
  xorb03aa1n02x5               g050(.a(new_n145), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n08x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nand42aa1n06x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  aoi012aa1n02x5               g053(.a(new_n147), .b(new_n145), .c(new_n148), .o1(new_n149));
  xnrb03aa1n02x5               g054(.a(new_n149), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n06x5               g055(.a(\b[13] ), .b(\a[14] ), .o1(new_n151));
  nand02aa1n06x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nona23aa1n03x5               g057(.a(new_n152), .b(new_n148), .c(new_n147), .d(new_n151), .out0(new_n153));
  tech160nm_fiaoi012aa1n05x5   g058(.a(new_n151), .b(new_n147), .c(new_n152), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n153), .c(new_n141), .d(new_n144), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  xorc02aa1n12x5               g062(.a(\a[15] ), .b(\b[14] ), .out0(new_n158));
  tech160nm_fixorc02aa1n02p5x5 g063(.a(\a[16] ), .b(\b[15] ), .out0(new_n159));
  aoi112aa1n02x5               g064(.a(new_n159), .b(new_n157), .c(new_n155), .d(new_n158), .o1(new_n160));
  aoai13aa1n04x5               g065(.a(new_n159), .b(new_n157), .c(new_n155), .d(new_n158), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(\s[16] ));
  nano23aa1n02x5               g067(.a(new_n131), .b(new_n133), .c(new_n134), .d(new_n132), .out0(new_n163));
  nano23aa1n02x4               g068(.a(new_n147), .b(new_n151), .c(new_n152), .d(new_n148), .out0(new_n164));
  nanp03aa1n02x5               g069(.a(new_n164), .b(new_n158), .c(new_n159), .o1(new_n165));
  nano32aa1n03x7               g070(.a(new_n165), .b(new_n163), .c(new_n123), .d(new_n97), .out0(new_n166));
  aoai13aa1n12x5               g071(.a(new_n166), .b(new_n122), .c(new_n117), .d(new_n107), .o1(new_n167));
  xnrc02aa1n02x5               g072(.a(\b[14] ), .b(\a[15] ), .out0(new_n168));
  xnrc02aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .out0(new_n169));
  nor043aa1n02x5               g074(.a(new_n153), .b(new_n169), .c(new_n168), .o1(new_n170));
  aob012aa1n02x5               g075(.a(new_n157), .b(\b[15] ), .c(\a[16] ), .out0(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  nanb03aa1n03x5               g077(.a(new_n154), .b(new_n159), .c(new_n158), .out0(new_n173));
  tech160nm_fioai012aa1n04x5   g078(.a(new_n173), .b(\b[15] ), .c(\a[16] ), .o1(new_n174));
  aoi112aa1n09x5               g079(.a(new_n174), .b(new_n172), .c(new_n143), .d(new_n170), .o1(new_n175));
  nand02aa1d08x5               g080(.a(new_n167), .b(new_n175), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g082(.a(\a[18] ), .o1(new_n178));
  inv040aa1d30x5               g083(.a(\a[17] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  xroi22aa1d06x4               g087(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  nor002aa1n02x5               g089(.a(\b[17] ), .b(\a[18] ), .o1(new_n185));
  nanp02aa1n04x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  aoi013aa1n06x4               g091(.a(new_n185), .b(new_n186), .c(new_n179), .d(new_n180), .o1(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n184), .c(new_n167), .d(new_n175), .o1(new_n188));
  xorb03aa1n02x5               g093(.a(new_n188), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g094(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n08x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nand42aa1n06x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  norp02aa1n12x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  tech160nm_finand02aa1n05x5   g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoi112aa1n03x5               g100(.a(new_n191), .b(new_n195), .c(new_n188), .d(new_n192), .o1(new_n196));
  aoai13aa1n03x5               g101(.a(new_n195), .b(new_n191), .c(new_n188), .d(new_n192), .o1(new_n197));
  norb02aa1n03x4               g102(.a(new_n197), .b(new_n196), .out0(\s[20] ));
  nano23aa1n09x5               g103(.a(new_n191), .b(new_n193), .c(new_n194), .d(new_n192), .out0(new_n199));
  nand22aa1n03x5               g104(.a(new_n183), .b(new_n199), .o1(new_n200));
  nona23aa1n09x5               g105(.a(new_n194), .b(new_n192), .c(new_n191), .d(new_n193), .out0(new_n201));
  tech160nm_fioai012aa1n03p5x5 g106(.a(new_n194), .b(new_n193), .c(new_n191), .o1(new_n202));
  oai012aa1n18x5               g107(.a(new_n202), .b(new_n201), .c(new_n187), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n200), .c(new_n167), .d(new_n175), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g111(.a(\b[20] ), .b(\a[21] ), .o1(new_n207));
  xorc02aa1n02x5               g112(.a(\a[21] ), .b(\b[20] ), .out0(new_n208));
  xorc02aa1n02x5               g113(.a(\a[22] ), .b(\b[21] ), .out0(new_n209));
  aoi112aa1n02x5               g114(.a(new_n207), .b(new_n209), .c(new_n205), .d(new_n208), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n209), .b(new_n207), .c(new_n205), .d(new_n208), .o1(new_n211));
  norb02aa1n02x7               g116(.a(new_n211), .b(new_n210), .out0(\s[22] ));
  inv000aa1d42x5               g117(.a(\a[21] ), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\a[22] ), .o1(new_n214));
  xroi22aa1d04x5               g119(.a(new_n213), .b(\b[20] ), .c(new_n214), .d(\b[21] ), .out0(new_n215));
  nanp03aa1n02x5               g120(.a(new_n215), .b(new_n183), .c(new_n199), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\b[21] ), .o1(new_n217));
  oao003aa1n02x5               g122(.a(new_n214), .b(new_n217), .c(new_n207), .carry(new_n218));
  aoi012aa1n02x5               g123(.a(new_n218), .b(new_n203), .c(new_n215), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n216), .c(new_n167), .d(new_n175), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g126(.a(\b[22] ), .b(\a[23] ), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[23] ), .b(\b[22] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[24] ), .b(\b[23] ), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n224), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n226));
  norb02aa1n02x7               g131(.a(new_n226), .b(new_n225), .out0(\s[24] ));
  inv030aa1d32x5               g132(.a(\a[23] ), .o1(new_n228));
  inv040aa1d32x5               g133(.a(\a[24] ), .o1(new_n229));
  xroi22aa1d06x4               g134(.a(new_n228), .b(\b[22] ), .c(new_n229), .d(\b[23] ), .out0(new_n230));
  nano22aa1n06x5               g135(.a(new_n200), .b(new_n215), .c(new_n230), .out0(new_n231));
  nona22aa1n02x4               g136(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(new_n232));
  oaib12aa1n02x5               g137(.a(new_n232), .b(\b[17] ), .c(new_n178), .out0(new_n233));
  inv040aa1n03x5               g138(.a(new_n202), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n215), .b(new_n234), .c(new_n199), .d(new_n233), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n218), .o1(new_n236));
  inv000aa1n02x5               g141(.a(new_n230), .o1(new_n237));
  oai022aa1n02x5               g142(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n238));
  oaib12aa1n02x5               g143(.a(new_n238), .b(new_n229), .c(\b[23] ), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n236), .o1(new_n240));
  xorc02aa1n02x5               g145(.a(\a[25] ), .b(\b[24] ), .out0(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n240), .c(new_n176), .d(new_n231), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n241), .b(new_n240), .c(new_n176), .d(new_n231), .o1(new_n243));
  norb02aa1n02x7               g148(.a(new_n242), .b(new_n243), .out0(\s[25] ));
  nor042aa1n03x5               g149(.a(\b[24] ), .b(\a[25] ), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[26] ), .b(\b[25] ), .out0(new_n246));
  nona22aa1n02x5               g151(.a(new_n242), .b(new_n246), .c(new_n245), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n245), .o1(new_n248));
  aobi12aa1n06x5               g153(.a(new_n246), .b(new_n242), .c(new_n248), .out0(new_n249));
  norb02aa1n03x4               g154(.a(new_n247), .b(new_n249), .out0(\s[26] ));
  inv000aa1d42x5               g155(.a(\a[25] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(\a[26] ), .o1(new_n252));
  xroi22aa1d06x4               g157(.a(new_n251), .b(\b[24] ), .c(new_n252), .d(\b[25] ), .out0(new_n253));
  nano32aa1n03x7               g158(.a(new_n200), .b(new_n253), .c(new_n215), .d(new_n230), .out0(new_n254));
  nand02aa1d06x5               g159(.a(new_n176), .b(new_n254), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[26] ), .b(\b[25] ), .c(new_n248), .carry(new_n256));
  aobi12aa1n06x5               g161(.a(new_n256), .b(new_n240), .c(new_n253), .out0(new_n257));
  xorc02aa1n02x5               g162(.a(\a[27] ), .b(\b[26] ), .out0(new_n258));
  xnbna2aa1n03x5               g163(.a(new_n258), .b(new_n257), .c(new_n255), .out0(\s[27] ));
  norp02aa1n02x5               g164(.a(\b[26] ), .b(\a[27] ), .o1(new_n260));
  inv040aa1n03x5               g165(.a(new_n260), .o1(new_n261));
  aobi12aa1n06x5               g166(.a(new_n258), .b(new_n257), .c(new_n255), .out0(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[27] ), .b(\a[28] ), .out0(new_n263));
  nano22aa1n03x5               g168(.a(new_n262), .b(new_n261), .c(new_n263), .out0(new_n264));
  aobi12aa1n06x5               g169(.a(new_n254), .b(new_n167), .c(new_n175), .out0(new_n265));
  aoai13aa1n02x7               g170(.a(new_n230), .b(new_n218), .c(new_n203), .d(new_n215), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n253), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n256), .b(new_n267), .c(new_n266), .d(new_n239), .o1(new_n268));
  oaih12aa1n02x5               g173(.a(new_n258), .b(new_n268), .c(new_n265), .o1(new_n269));
  tech160nm_fiaoi012aa1n02p5x5 g174(.a(new_n263), .b(new_n269), .c(new_n261), .o1(new_n270));
  norp02aa1n03x5               g175(.a(new_n270), .b(new_n264), .o1(\s[28] ));
  norb02aa1n02x5               g176(.a(new_n258), .b(new_n263), .out0(new_n272));
  aobi12aa1n02x7               g177(.a(new_n272), .b(new_n257), .c(new_n255), .out0(new_n273));
  oao003aa1n02x5               g178(.a(\a[28] ), .b(\b[27] ), .c(new_n261), .carry(new_n274));
  xnrc02aa1n02x5               g179(.a(\b[28] ), .b(\a[29] ), .out0(new_n275));
  nano22aa1n03x5               g180(.a(new_n273), .b(new_n274), .c(new_n275), .out0(new_n276));
  oaih12aa1n02x5               g181(.a(new_n272), .b(new_n268), .c(new_n265), .o1(new_n277));
  tech160nm_fiaoi012aa1n02p5x5 g182(.a(new_n275), .b(new_n277), .c(new_n274), .o1(new_n278));
  norp02aa1n03x5               g183(.a(new_n278), .b(new_n276), .o1(\s[29] ));
  xorb03aa1n02x5               g184(.a(new_n115), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g185(.a(new_n258), .b(new_n275), .c(new_n263), .out0(new_n281));
  aobi12aa1n02x7               g186(.a(new_n281), .b(new_n257), .c(new_n255), .out0(new_n282));
  oao003aa1n02x5               g187(.a(\a[29] ), .b(\b[28] ), .c(new_n274), .carry(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[29] ), .b(\a[30] ), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n282), .b(new_n283), .c(new_n284), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n281), .b(new_n268), .c(new_n265), .o1(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n284), .b(new_n286), .c(new_n283), .o1(new_n287));
  norp02aa1n03x5               g192(.a(new_n287), .b(new_n285), .o1(\s[30] ));
  norb02aa1n02x5               g193(.a(new_n281), .b(new_n284), .out0(new_n289));
  aobi12aa1n06x5               g194(.a(new_n289), .b(new_n257), .c(new_n255), .out0(new_n290));
  oao003aa1n02x5               g195(.a(\a[30] ), .b(\b[29] ), .c(new_n283), .carry(new_n291));
  xnrc02aa1n02x5               g196(.a(\b[30] ), .b(\a[31] ), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n290), .b(new_n291), .c(new_n292), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n289), .b(new_n268), .c(new_n265), .o1(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n292), .b(new_n294), .c(new_n291), .o1(new_n295));
  norp02aa1n03x5               g200(.a(new_n295), .b(new_n293), .o1(\s[31] ));
  xnrb03aa1n02x5               g201(.a(new_n116), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g202(.a(\a[3] ), .b(\b[2] ), .c(new_n116), .o1(new_n298));
  xorb03aa1n02x5               g203(.a(new_n298), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g204(.a(new_n117), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norp02aa1n02x5               g205(.a(\b[4] ), .b(\a[5] ), .o1(new_n301));
  aoi012aa1n02x5               g206(.a(new_n301), .b(new_n117), .c(new_n106), .o1(new_n302));
  xnrb03aa1n02x5               g207(.a(new_n302), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g208(.a(\a[6] ), .b(\b[5] ), .c(new_n302), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g210(.a(new_n102), .b(new_n304), .c(new_n103), .o1(new_n306));
  xnbna2aa1n03x5               g211(.a(new_n306), .b(new_n118), .c(new_n101), .out0(\s[8] ));
  aoi112aa1n02x5               g212(.a(new_n122), .b(new_n123), .c(new_n117), .d(new_n107), .o1(new_n308));
  norb02aa1n02x5               g213(.a(new_n124), .b(new_n308), .out0(\s[9] ));
endmodule


