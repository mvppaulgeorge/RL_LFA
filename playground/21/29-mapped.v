// Benchmark "adder" written by ABC on Wed Jul 17 22:58:40 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n182, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n310, new_n311, new_n313,
    new_n314, new_n315, new_n316, new_n318;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanb02aa1n12x5               g003(.a(\a[9] ), .b(new_n98), .out0(new_n99));
  nor042aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand02aa1d24x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor042aa1n09x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand02aa1n16x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nano23aa1n06x5               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[6] ), .o1(new_n105));
  inv030aa1n06x5               g010(.a(\b[5] ), .o1(new_n106));
  nor042aa1n06x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  oao003aa1n03x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .carry(new_n108));
  inv000aa1n02x5               g013(.a(new_n102), .o1(new_n109));
  oaoi03aa1n03x5               g014(.a(\a[8] ), .b(\b[7] ), .c(new_n109), .o1(new_n110));
  tech160nm_fiaoi012aa1n03p5x5 g015(.a(new_n110), .b(new_n104), .c(new_n108), .o1(new_n111));
  nor042aa1d18x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  inv040aa1n02x5               g017(.a(new_n112), .o1(new_n113));
  oao003aa1n06x5               g018(.a(\a[4] ), .b(\b[3] ), .c(new_n113), .carry(new_n114));
  xorc02aa1n03x5               g019(.a(\a[4] ), .b(\b[3] ), .out0(new_n115));
  nand02aa1n04x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  norb02aa1n03x5               g021(.a(new_n116), .b(new_n112), .out0(new_n117));
  and002aa1n12x5               g022(.a(\b[1] ), .b(\a[2] ), .o(new_n118));
  nand22aa1n12x5               g023(.a(\b[0] ), .b(\a[1] ), .o1(new_n119));
  nor042aa1n04x5               g024(.a(\b[1] ), .b(\a[2] ), .o1(new_n120));
  oab012aa1n09x5               g025(.a(new_n118), .b(new_n120), .c(new_n119), .out0(new_n121));
  nand23aa1n06x5               g026(.a(new_n121), .b(new_n115), .c(new_n117), .o1(new_n122));
  tech160nm_fixorc02aa1n02p5x5 g027(.a(\a[6] ), .b(\b[5] ), .out0(new_n123));
  xorc02aa1n12x5               g028(.a(\a[5] ), .b(\b[4] ), .out0(new_n124));
  nand03aa1n02x5               g029(.a(new_n104), .b(new_n123), .c(new_n124), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n111), .b(new_n125), .c(new_n122), .d(new_n114), .o1(new_n126));
  oaib12aa1n06x5               g031(.a(new_n126), .b(new_n98), .c(\a[9] ), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n97), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  and002aa1n02x5               g033(.a(\b[9] ), .b(\a[10] ), .o(new_n129));
  aoi013aa1n06x4               g034(.a(new_n129), .b(new_n127), .c(new_n99), .d(new_n97), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n20x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor042aa1n04x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nanp02aa1n12x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n133), .o1(new_n138));
  aoi112aa1n02x5               g043(.a(new_n137), .b(new_n132), .c(new_n130), .d(new_n133), .o1(new_n139));
  nanb02aa1n03x5               g044(.a(new_n139), .b(new_n138), .out0(\s[12] ));
  nano23aa1d15x5               g045(.a(new_n132), .b(new_n134), .c(new_n135), .d(new_n133), .out0(new_n141));
  tech160nm_fioaoi03aa1n03p5x5 g046(.a(\a[10] ), .b(\b[9] ), .c(new_n99), .o1(new_n142));
  ao0012aa1n06x5               g047(.a(new_n134), .b(new_n132), .c(new_n135), .o(new_n143));
  tech160nm_fiao0012aa1n02p5x5 g048(.a(new_n143), .b(new_n141), .c(new_n142), .o(new_n144));
  xorc02aa1n12x5               g049(.a(\a[9] ), .b(\b[8] ), .out0(new_n145));
  nand23aa1n09x5               g050(.a(new_n141), .b(new_n97), .c(new_n145), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  xorc02aa1n02x5               g052(.a(\a[13] ), .b(\b[12] ), .out0(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n144), .c(new_n126), .d(new_n147), .o1(new_n149));
  aoi112aa1n02x5               g054(.a(new_n144), .b(new_n148), .c(new_n126), .d(new_n147), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(\s[13] ));
  orn002aa1n24x5               g056(.a(\a[13] ), .b(\b[12] ), .o(new_n152));
  xnrc02aa1n12x5               g057(.a(\b[13] ), .b(\a[14] ), .out0(new_n153));
  xobna2aa1n03x5               g058(.a(new_n153), .b(new_n149), .c(new_n152), .out0(\s[14] ));
  nanp02aa1n02x5               g059(.a(new_n126), .b(new_n147), .o1(new_n155));
  nand42aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nano22aa1d15x5               g061(.a(new_n153), .b(new_n152), .c(new_n156), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n12x5               g063(.a(new_n157), .b(new_n143), .c(new_n141), .d(new_n142), .o1(new_n159));
  oao003aa1n02x5               g064(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .carry(new_n160));
  nand42aa1n04x5               g065(.a(new_n159), .b(new_n160), .o1(new_n161));
  oabi12aa1n03x5               g066(.a(new_n161), .b(new_n155), .c(new_n158), .out0(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1d32x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand02aa1d08x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  xnrc02aa1n12x5               g070(.a(\b[15] ), .b(\a[16] ), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n164), .b(new_n166), .c(new_n162), .d(new_n165), .o1(new_n168));
  nanb02aa1n03x5               g073(.a(new_n168), .b(new_n167), .out0(\s[16] ));
  inv000aa1d42x5               g074(.a(\a[17] ), .o1(new_n170));
  nanb02aa1n12x5               g075(.a(new_n164), .b(new_n165), .out0(new_n171));
  nor042aa1n06x5               g076(.a(new_n166), .b(new_n171), .o1(new_n172));
  nano22aa1n12x5               g077(.a(new_n146), .b(new_n157), .c(new_n172), .out0(new_n173));
  inv000aa1n02x5               g078(.a(new_n172), .o1(new_n174));
  aoi012aa1n12x5               g079(.a(new_n174), .b(new_n159), .c(new_n160), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\a[16] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\b[15] ), .o1(new_n177));
  oaoi03aa1n02x5               g082(.a(new_n176), .b(new_n177), .c(new_n164), .o1(new_n178));
  inv000aa1n02x5               g083(.a(new_n178), .o1(new_n179));
  aoi112aa1n09x5               g084(.a(new_n175), .b(new_n179), .c(new_n126), .d(new_n173), .o1(new_n180));
  xorb03aa1n02x5               g085(.a(new_n180), .b(\b[16] ), .c(new_n170), .out0(\s[17] ));
  oaoi03aa1n03x5               g086(.a(\a[17] ), .b(\b[16] ), .c(new_n180), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv040aa1d32x5               g088(.a(\a[18] ), .o1(new_n184));
  xroi22aa1d06x4               g089(.a(new_n170), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n185));
  inv030aa1n02x5               g090(.a(new_n185), .o1(new_n186));
  aoi112aa1n06x5               g091(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n187));
  aoib12aa1n06x5               g092(.a(new_n187), .b(new_n184), .c(\b[17] ), .out0(new_n188));
  tech160nm_fioai012aa1n05x5   g093(.a(new_n188), .b(new_n180), .c(new_n186), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g095(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nand22aa1n04x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  nor022aa1n08x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  nand22aa1n12x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  aoai13aa1n03x5               g102(.a(new_n197), .b(new_n192), .c(new_n189), .d(new_n194), .o1(new_n198));
  nanp02aa1n06x5               g103(.a(new_n126), .b(new_n173), .o1(new_n199));
  nand22aa1n02x5               g104(.a(new_n161), .b(new_n172), .o1(new_n200));
  nand23aa1n06x5               g105(.a(new_n199), .b(new_n200), .c(new_n178), .o1(new_n201));
  nor042aa1n03x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  aob012aa1n02x5               g107(.a(new_n202), .b(\b[17] ), .c(\a[18] ), .out0(new_n203));
  oaib12aa1n09x5               g108(.a(new_n203), .b(\b[17] ), .c(new_n184), .out0(new_n204));
  aoai13aa1n03x5               g109(.a(new_n194), .b(new_n204), .c(new_n201), .d(new_n185), .o1(new_n205));
  nona22aa1n02x4               g110(.a(new_n205), .b(new_n197), .c(new_n192), .out0(new_n206));
  nanp02aa1n03x5               g111(.a(new_n198), .b(new_n206), .o1(\s[20] ));
  nona23aa1d18x5               g112(.a(new_n196), .b(new_n193), .c(new_n192), .d(new_n195), .out0(new_n208));
  ao0012aa1n12x5               g113(.a(new_n195), .b(new_n192), .c(new_n196), .o(new_n209));
  oabi12aa1n18x5               g114(.a(new_n209), .b(new_n208), .c(new_n188), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n185), .b(new_n208), .out0(new_n212));
  inv000aa1n02x5               g117(.a(new_n212), .o1(new_n213));
  tech160nm_fioai012aa1n05x5   g118(.a(new_n211), .b(new_n180), .c(new_n213), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xnrc02aa1n12x5               g121(.a(\b[20] ), .b(\a[21] ), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  xnrc02aa1n12x5               g123(.a(\b[21] ), .b(\a[22] ), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n216), .c(new_n214), .d(new_n218), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n218), .b(new_n210), .c(new_n201), .d(new_n212), .o1(new_n221));
  nona22aa1n03x5               g126(.a(new_n221), .b(new_n219), .c(new_n216), .out0(new_n222));
  nanp02aa1n03x5               g127(.a(new_n220), .b(new_n222), .o1(\s[22] ));
  nor042aa1n06x5               g128(.a(new_n219), .b(new_n217), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[21] ), .o1(new_n226));
  oaoi03aa1n09x5               g131(.a(new_n225), .b(new_n226), .c(new_n216), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoi012aa1d24x5               g133(.a(new_n228), .b(new_n210), .c(new_n224), .o1(new_n229));
  nano23aa1n06x5               g134(.a(new_n192), .b(new_n195), .c(new_n196), .d(new_n193), .out0(new_n230));
  nano22aa1d15x5               g135(.a(new_n186), .b(new_n224), .c(new_n230), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  tech160nm_fioai012aa1n05x5   g137(.a(new_n229), .b(new_n180), .c(new_n232), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n12x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  tech160nm_fixnrc02aa1n05x5   g141(.a(\b[23] ), .b(\a[24] ), .out0(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n229), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n236), .b(new_n239), .c(new_n201), .d(new_n231), .o1(new_n240));
  nona22aa1n02x4               g145(.a(new_n240), .b(new_n237), .c(new_n235), .out0(new_n241));
  nanp02aa1n03x5               g146(.a(new_n238), .b(new_n241), .o1(\s[24] ));
  norb02aa1n03x4               g147(.a(new_n236), .b(new_n237), .out0(new_n243));
  inv040aa1n02x5               g148(.a(new_n243), .o1(new_n244));
  nano32aa1n02x4               g149(.a(new_n244), .b(new_n185), .c(new_n224), .d(new_n230), .out0(new_n245));
  inv000aa1n02x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n224), .b(new_n209), .c(new_n230), .d(new_n204), .o1(new_n247));
  orn002aa1n02x5               g152(.a(\a[23] ), .b(\b[22] ), .o(new_n248));
  oao003aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .c(new_n248), .carry(new_n249));
  aoai13aa1n12x5               g154(.a(new_n249), .b(new_n244), .c(new_n247), .d(new_n227), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  tech160nm_fioai012aa1n05x5   g156(.a(new_n251), .b(new_n180), .c(new_n246), .o1(new_n252));
  xorb03aa1n02x5               g157(.a(new_n252), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  xorc02aa1n06x5               g159(.a(\a[25] ), .b(\b[24] ), .out0(new_n255));
  xnrc02aa1n02x5               g160(.a(\b[25] ), .b(\a[26] ), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n254), .c(new_n252), .d(new_n255), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n255), .b(new_n250), .c(new_n201), .d(new_n245), .o1(new_n258));
  nona22aa1n03x5               g163(.a(new_n258), .b(new_n256), .c(new_n254), .out0(new_n259));
  nanp02aa1n03x5               g164(.a(new_n257), .b(new_n259), .o1(\s[26] ));
  norb02aa1n02x5               g165(.a(new_n255), .b(new_n256), .out0(new_n261));
  inv000aa1d42x5               g166(.a(\a[26] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(\b[25] ), .o1(new_n263));
  oaoi03aa1n09x5               g168(.a(new_n262), .b(new_n263), .c(new_n254), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  tech160nm_fiaoi012aa1n05x5   g170(.a(new_n265), .b(new_n250), .c(new_n261), .o1(new_n266));
  inv000aa1n02x5               g171(.a(new_n261), .o1(new_n267));
  nona22aa1n03x5               g172(.a(new_n231), .b(new_n267), .c(new_n244), .out0(new_n268));
  tech160nm_fioai012aa1n05x5   g173(.a(new_n266), .b(new_n180), .c(new_n268), .o1(new_n269));
  xorb03aa1n03x5               g174(.a(new_n269), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .d(new_n272), .o1(new_n274));
  aoai13aa1n09x5               g179(.a(new_n243), .b(new_n228), .c(new_n210), .d(new_n224), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n264), .b(new_n267), .c(new_n275), .d(new_n249), .o1(new_n276));
  inv040aa1n02x5               g181(.a(new_n268), .o1(new_n277));
  aoai13aa1n02x5               g182(.a(new_n272), .b(new_n276), .c(new_n201), .d(new_n277), .o1(new_n278));
  nona22aa1n03x5               g183(.a(new_n278), .b(new_n273), .c(new_n271), .out0(new_n279));
  nanp02aa1n03x5               g184(.a(new_n274), .b(new_n279), .o1(\s[28] ));
  norb02aa1n02x5               g185(.a(new_n272), .b(new_n273), .out0(new_n281));
  aoai13aa1n02x5               g186(.a(new_n281), .b(new_n276), .c(new_n201), .d(new_n277), .o1(new_n282));
  aob012aa1n03x5               g187(.a(new_n271), .b(\b[27] ), .c(\a[28] ), .out0(new_n283));
  oa0012aa1n12x5               g188(.a(new_n283), .b(\b[27] ), .c(\a[28] ), .o(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[28] ), .b(\a[29] ), .out0(new_n286));
  nona22aa1n02x4               g191(.a(new_n282), .b(new_n285), .c(new_n286), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n286), .b(new_n285), .c(new_n269), .d(new_n281), .o1(new_n288));
  nanp02aa1n03x5               g193(.a(new_n288), .b(new_n287), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n119), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n272), .b(new_n286), .c(new_n273), .out0(new_n291));
  oaoi03aa1n09x5               g196(.a(\a[29] ), .b(\b[28] ), .c(new_n284), .o1(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[29] ), .b(\a[30] ), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n292), .c(new_n269), .d(new_n291), .o1(new_n294));
  aoai13aa1n02x5               g199(.a(new_n291), .b(new_n276), .c(new_n201), .d(new_n277), .o1(new_n295));
  nona22aa1n03x5               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[30] ));
  nanb02aa1n02x5               g202(.a(new_n293), .b(new_n292), .out0(new_n298));
  oai012aa1n02x5               g203(.a(new_n298), .b(\b[29] ), .c(\a[30] ), .o1(new_n299));
  norb02aa1n02x5               g204(.a(new_n291), .b(new_n293), .out0(new_n300));
  aoai13aa1n02x7               g205(.a(new_n300), .b(new_n276), .c(new_n201), .d(new_n277), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nona22aa1n02x4               g207(.a(new_n301), .b(new_n302), .c(new_n299), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n302), .b(new_n299), .c(new_n269), .d(new_n300), .o1(new_n304));
  nanp02aa1n03x5               g209(.a(new_n304), .b(new_n303), .o1(\s[31] ));
  xobna2aa1n03x5               g210(.a(new_n121), .b(new_n116), .c(new_n113), .out0(\s[3] ));
  oai012aa1n02x5               g211(.a(new_n116), .b(new_n121), .c(new_n112), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xnbna2aa1n03x5               g213(.a(new_n124), .b(new_n122), .c(new_n114), .out0(\s[5] ));
  nanp02aa1n02x5               g214(.a(new_n122), .b(new_n114), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n107), .b(new_n310), .c(new_n124), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(new_n105), .out0(\s[6] ));
  norb02aa1n02x5               g217(.a(new_n103), .b(new_n102), .out0(new_n313));
  nanp02aa1n02x5               g218(.a(new_n311), .b(new_n123), .o1(new_n314));
  oai112aa1n02x5               g219(.a(new_n314), .b(new_n313), .c(new_n106), .d(new_n105), .o1(new_n315));
  oaoi13aa1n02x5               g220(.a(new_n313), .b(new_n314), .c(new_n105), .d(new_n106), .o1(new_n316));
  norb02aa1n02x5               g221(.a(new_n315), .b(new_n316), .out0(\s[7] ));
  norb02aa1n02x5               g222(.a(new_n101), .b(new_n100), .out0(new_n318));
  xnbna2aa1n03x5               g223(.a(new_n318), .b(new_n315), .c(new_n109), .out0(\s[8] ));
  xorb03aa1n02x5               g224(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


