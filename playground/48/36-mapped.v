// Benchmark "adder" written by ABC on Thu Jul 18 12:58:25 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n323, new_n324, new_n326, new_n327,
    new_n328, new_n330, new_n332;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n03x5               g003(.a(\b[3] ), .b(\a[4] ), .o1(new_n99));
  nor022aa1n08x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  norp02aa1n06x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  tech160nm_fioai012aa1n03p5x5 g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nand02aa1d04x5               g008(.a(\b[0] ), .b(\a[1] ), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  tech160nm_fioai012aa1n05x5   g010(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n106));
  nand42aa1n03x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n99), .b(new_n107), .c(new_n101), .d(new_n100), .out0(new_n108));
  oai012aa1n12x5               g013(.a(new_n102), .b(new_n108), .c(new_n106), .o1(new_n109));
  tech160nm_fixnrc02aa1n04x5   g014(.a(\b[5] ), .b(\a[6] ), .out0(new_n110));
  xnrc02aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .out0(new_n111));
  norp02aa1n12x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand02aa1n08x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1d32x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1d18x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  nor043aa1n03x5               g021(.a(new_n116), .b(new_n111), .c(new_n110), .o1(new_n117));
  norb03aa1n03x5               g022(.a(new_n113), .b(new_n112), .c(new_n114), .out0(new_n118));
  oai022aa1d24x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  aob012aa1n12x5               g024(.a(new_n119), .b(\b[5] ), .c(\a[6] ), .out0(new_n120));
  obai22aa1n09x5               g025(.a(new_n113), .b(new_n118), .c(new_n116), .d(new_n120), .out0(new_n121));
  nand02aa1d08x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  norb02aa1n02x5               g027(.a(new_n122), .b(new_n97), .out0(new_n123));
  aoai13aa1n02x7               g028(.a(new_n123), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n124));
  nor042aa1n09x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand02aa1d16x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n127), .b(new_n124), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g033(.a(new_n127), .o1(new_n129));
  oai012aa1n06x5               g034(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n129), .c(new_n124), .d(new_n98), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor022aa1n16x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  nor002aa1d32x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand42aa1d28x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  aoai13aa1n03x5               g043(.a(new_n138), .b(new_n133), .c(new_n131), .d(new_n135), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n137), .b(new_n136), .c(new_n133), .out0(new_n140));
  aoai13aa1n02x5               g045(.a(new_n139), .b(new_n140), .c(new_n135), .d(new_n131), .o1(\s[12] ));
  nano23aa1n09x5               g046(.a(new_n133), .b(new_n136), .c(new_n137), .d(new_n134), .out0(new_n142));
  nano23aa1n06x5               g047(.a(new_n97), .b(new_n125), .c(new_n126), .d(new_n122), .out0(new_n143));
  nand22aa1n09x5               g048(.a(new_n143), .b(new_n142), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n146));
  nona23aa1n09x5               g051(.a(new_n137), .b(new_n134), .c(new_n133), .d(new_n136), .out0(new_n147));
  oaih12aa1n02x5               g052(.a(new_n137), .b(new_n136), .c(new_n133), .o1(new_n148));
  oai012aa1n09x5               g053(.a(new_n148), .b(new_n147), .c(new_n130), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n149), .b(new_n146), .out0(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  inv000aa1n02x5               g057(.a(new_n152), .o1(new_n153));
  nand42aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanp03aa1n02x5               g059(.a(new_n150), .b(new_n153), .c(new_n154), .o1(new_n155));
  nor002aa1n03x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1n06x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  oai022aa1n02x5               g063(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n159));
  nanb03aa1n02x5               g064(.a(new_n159), .b(new_n155), .c(new_n157), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n160), .b(new_n158), .c(new_n153), .d(new_n155), .o1(\s[14] ));
  nona23aa1n02x4               g066(.a(new_n157), .b(new_n154), .c(new_n152), .d(new_n156), .out0(new_n162));
  nano23aa1n09x5               g067(.a(new_n152), .b(new_n156), .c(new_n157), .d(new_n154), .out0(new_n163));
  aoi022aa1n02x5               g068(.a(new_n149), .b(new_n163), .c(new_n157), .d(new_n159), .o1(new_n164));
  oaih12aa1n02x5               g069(.a(new_n164), .b(new_n146), .c(new_n162), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  inv000aa1n06x5               g072(.a(new_n167), .o1(new_n168));
  nand02aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanp03aa1n02x5               g074(.a(new_n165), .b(new_n168), .c(new_n169), .o1(new_n170));
  nor042aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nand42aa1n06x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  nona23aa1n02x4               g078(.a(new_n170), .b(new_n172), .c(new_n171), .d(new_n167), .out0(new_n174));
  aoai13aa1n03x5               g079(.a(new_n174), .b(new_n173), .c(new_n170), .d(new_n168), .o1(\s[16] ));
  nano23aa1n06x5               g080(.a(new_n167), .b(new_n171), .c(new_n172), .d(new_n169), .out0(new_n176));
  nano22aa1n03x7               g081(.a(new_n144), .b(new_n163), .c(new_n176), .out0(new_n177));
  aoai13aa1n12x5               g082(.a(new_n177), .b(new_n121), .c(new_n109), .d(new_n117), .o1(new_n178));
  oaoi03aa1n02x5               g083(.a(\a[14] ), .b(\b[13] ), .c(new_n153), .o1(new_n179));
  aoai13aa1n12x5               g084(.a(new_n176), .b(new_n179), .c(new_n149), .d(new_n163), .o1(new_n180));
  oaoi03aa1n02x5               g085(.a(\a[16] ), .b(\b[15] ), .c(new_n168), .o1(new_n181));
  inv000aa1n02x5               g086(.a(new_n181), .o1(new_n182));
  nand23aa1d12x5               g087(.a(new_n178), .b(new_n180), .c(new_n182), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  xorc02aa1n02x5               g089(.a(\a[18] ), .b(\b[17] ), .out0(new_n185));
  inv000aa1d42x5               g090(.a(\a[17] ), .o1(new_n186));
  inv040aa1d32x5               g091(.a(\b[16] ), .o1(new_n187));
  oaoi03aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n183), .o1(new_n188));
  oaoi13aa1n02x5               g093(.a(new_n162), .b(new_n148), .c(new_n147), .d(new_n130), .o1(new_n189));
  oaoi13aa1n02x7               g094(.a(new_n181), .b(new_n176), .c(new_n189), .d(new_n179), .o1(new_n190));
  xnrc02aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .out0(new_n191));
  oaih22aa1d12x5               g096(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n192));
  aoi012aa1n02x5               g097(.a(new_n192), .b(\a[18] ), .c(\b[17] ), .o1(new_n193));
  aoai13aa1n02x5               g098(.a(new_n193), .b(new_n191), .c(new_n190), .d(new_n178), .o1(new_n194));
  oai012aa1n02x5               g099(.a(new_n194), .b(new_n188), .c(new_n185), .o1(\s[18] ));
  inv000aa1d42x5               g100(.a(\b[17] ), .o1(new_n196));
  xroi22aa1d06x4               g101(.a(new_n196), .b(\a[18] ), .c(new_n187), .d(\a[17] ), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  oaib12aa1n02x7               g103(.a(new_n192), .b(new_n196), .c(\a[18] ), .out0(new_n199));
  aoai13aa1n02x7               g104(.a(new_n199), .b(new_n198), .c(new_n190), .d(new_n178), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nand02aa1n06x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nor042aa1n09x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand42aa1n08x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n203), .c(new_n200), .d(new_n204), .o1(new_n208));
  nand22aa1n03x5               g113(.a(new_n183), .b(new_n197), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n203), .b(new_n204), .out0(new_n210));
  norb03aa1n02x5               g115(.a(new_n206), .b(new_n203), .c(new_n205), .out0(new_n211));
  aoai13aa1n03x5               g116(.a(new_n211), .b(new_n210), .c(new_n209), .d(new_n199), .o1(new_n212));
  nanp02aa1n03x5               g117(.a(new_n208), .b(new_n212), .o1(\s[20] ));
  nona23aa1n09x5               g118(.a(new_n206), .b(new_n204), .c(new_n203), .d(new_n205), .out0(new_n214));
  norb03aa1n02x5               g119(.a(new_n185), .b(new_n214), .c(new_n191), .out0(new_n215));
  oai012aa1n09x5               g120(.a(new_n206), .b(new_n205), .c(new_n203), .o1(new_n216));
  tech160nm_fioai012aa1n05x5   g121(.a(new_n216), .b(new_n214), .c(new_n199), .o1(new_n217));
  xnrc02aa1n12x5               g122(.a(\b[20] ), .b(\a[21] ), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n217), .c(new_n183), .d(new_n215), .o1(new_n220));
  aoi112aa1n02x5               g125(.a(new_n219), .b(new_n217), .c(new_n183), .d(new_n215), .o1(new_n221));
  norb02aa1n03x4               g126(.a(new_n220), .b(new_n221), .out0(\s[21] ));
  nor042aa1n03x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  inv040aa1n03x5               g128(.a(new_n223), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  and002aa1n02x5               g131(.a(\b[21] ), .b(\a[22] ), .o(new_n227));
  oai022aa1n02x5               g132(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n228));
  nona22aa1n02x5               g133(.a(new_n220), .b(new_n227), .c(new_n228), .out0(new_n229));
  aoai13aa1n03x5               g134(.a(new_n229), .b(new_n226), .c(new_n224), .d(new_n220), .o1(\s[22] ));
  nano23aa1d15x5               g135(.a(new_n203), .b(new_n205), .c(new_n206), .d(new_n204), .out0(new_n231));
  nor022aa1n12x5               g136(.a(new_n225), .b(new_n218), .o1(new_n232));
  nand23aa1d12x5               g137(.a(new_n197), .b(new_n232), .c(new_n231), .o1(new_n233));
  inv040aa1n02x5               g138(.a(new_n233), .o1(new_n234));
  inv000aa1n03x5               g139(.a(new_n199), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n216), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n232), .b(new_n236), .c(new_n231), .d(new_n235), .o1(new_n237));
  oaoi03aa1n02x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n224), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  nanp02aa1n02x5               g144(.a(new_n237), .b(new_n239), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[22] ), .b(\a[23] ), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n240), .c(new_n183), .d(new_n234), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(new_n242), .b(new_n240), .c(new_n183), .d(new_n234), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n243), .b(new_n244), .out0(\s[23] ));
  norp02aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .out0(new_n248));
  and002aa1n02x5               g153(.a(\b[23] ), .b(\a[24] ), .o(new_n249));
  oai022aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n250));
  nona22aa1n02x5               g155(.a(new_n243), .b(new_n249), .c(new_n250), .out0(new_n251));
  aoai13aa1n03x5               g156(.a(new_n251), .b(new_n248), .c(new_n247), .d(new_n243), .o1(\s[24] ));
  norb02aa1n06x4               g157(.a(new_n248), .b(new_n241), .out0(new_n253));
  inv030aa1n02x5               g158(.a(new_n253), .o1(new_n254));
  nano32aa1n03x7               g159(.a(new_n254), .b(new_n197), .c(new_n232), .d(new_n231), .out0(new_n255));
  aob012aa1n02x5               g160(.a(new_n250), .b(\b[23] ), .c(\a[24] ), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n254), .c(new_n237), .d(new_n239), .o1(new_n257));
  tech160nm_fixorc02aa1n03p5x5 g162(.a(\a[25] ), .b(\b[24] ), .out0(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n257), .c(new_n183), .d(new_n255), .o1(new_n259));
  aoi112aa1n02x5               g164(.a(new_n258), .b(new_n257), .c(new_n183), .d(new_n255), .o1(new_n260));
  norb02aa1n02x5               g165(.a(new_n259), .b(new_n260), .out0(\s[25] ));
  norp02aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  and002aa1n02x5               g169(.a(\b[25] ), .b(\a[26] ), .o(new_n265));
  oai022aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n266));
  nona22aa1n02x5               g171(.a(new_n259), .b(new_n265), .c(new_n266), .out0(new_n267));
  aoai13aa1n03x5               g172(.a(new_n267), .b(new_n264), .c(new_n263), .d(new_n259), .o1(\s[26] ));
  and002aa1n06x5               g173(.a(new_n264), .b(new_n258), .o(new_n269));
  nano22aa1d15x5               g174(.a(new_n233), .b(new_n269), .c(new_n253), .out0(new_n270));
  nand02aa1d06x5               g175(.a(new_n183), .b(new_n270), .o1(new_n271));
  aob012aa1n02x5               g176(.a(new_n266), .b(\b[25] ), .c(\a[26] ), .out0(new_n272));
  aobi12aa1n06x5               g177(.a(new_n272), .b(new_n257), .c(new_n269), .out0(new_n273));
  xorc02aa1n12x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n06x5               g179(.a(new_n274), .b(new_n271), .c(new_n273), .out0(\s[27] ));
  xorc02aa1n12x5               g180(.a(\a[28] ), .b(\b[27] ), .out0(new_n276));
  inv020aa1n04x5               g181(.a(new_n270), .o1(new_n277));
  aoi013aa1n03x5               g182(.a(new_n277), .b(new_n178), .c(new_n180), .d(new_n182), .o1(new_n278));
  aoai13aa1n04x5               g183(.a(new_n253), .b(new_n238), .c(new_n217), .d(new_n232), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n269), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n272), .b(new_n280), .c(new_n279), .d(new_n256), .o1(new_n281));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  oaoi13aa1n03x5               g187(.a(new_n282), .b(new_n274), .c(new_n278), .d(new_n281), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n274), .b(new_n281), .c(new_n183), .d(new_n270), .o1(new_n284));
  oai112aa1n03x5               g189(.a(new_n284), .b(new_n276), .c(\b[26] ), .d(\a[27] ), .o1(new_n285));
  oaih12aa1n02x5               g190(.a(new_n285), .b(new_n283), .c(new_n276), .o1(\s[28] ));
  and002aa1n12x5               g191(.a(new_n276), .b(new_n274), .o(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n281), .c(new_n183), .d(new_n270), .o1(new_n288));
  tech160nm_fixorc02aa1n02p5x5 g193(.a(\a[29] ), .b(\b[28] ), .out0(new_n289));
  inv000aa1d42x5               g194(.a(\a[28] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(\b[27] ), .o1(new_n291));
  aoi112aa1n09x5               g196(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n292));
  aoi112aa1n02x5               g197(.a(new_n289), .b(new_n292), .c(new_n290), .d(new_n291), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n287), .o1(new_n294));
  aoi012aa1n02x5               g199(.a(new_n292), .b(new_n290), .c(new_n291), .o1(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n294), .c(new_n271), .d(new_n273), .o1(new_n296));
  aoi022aa1n03x5               g201(.a(new_n296), .b(new_n289), .c(new_n288), .d(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n104), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g203(.a(new_n274), .b(new_n289), .c(new_n276), .o(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n281), .c(new_n183), .d(new_n270), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .out0(new_n301));
  norp02aa1n02x5               g206(.a(\b[28] ), .b(\a[29] ), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n295), .b(\a[29] ), .c(\b[28] ), .o1(new_n303));
  norp03aa1n02x5               g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n299), .o1(new_n305));
  oaoi03aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .o1(new_n306));
  inv000aa1n03x5               g211(.a(new_n306), .o1(new_n307));
  aoai13aa1n02x7               g212(.a(new_n307), .b(new_n305), .c(new_n271), .d(new_n273), .o1(new_n308));
  aoi022aa1n03x5               g213(.a(new_n308), .b(new_n301), .c(new_n300), .d(new_n304), .o1(\s[30] ));
  nano22aa1d24x5               g214(.a(new_n294), .b(new_n289), .c(new_n301), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n281), .c(new_n183), .d(new_n270), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[31] ), .b(\b[30] ), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n307), .carry(new_n313));
  norb02aa1n02x5               g218(.a(new_n313), .b(new_n312), .out0(new_n314));
  inv000aa1d42x5               g219(.a(new_n310), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n313), .b(new_n315), .c(new_n271), .d(new_n273), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n316), .b(new_n312), .c(new_n311), .d(new_n314), .o1(\s[31] ));
  xnrb03aa1n02x5               g222(.a(new_n106), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n106), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g226(.a(new_n111), .b(new_n109), .out0(new_n322));
  oaoi13aa1n06x5               g227(.a(new_n110), .b(new_n322), .c(\a[5] ), .d(\b[4] ), .o1(new_n323));
  oai112aa1n02x5               g228(.a(new_n322), .b(new_n110), .c(\b[4] ), .d(\a[5] ), .o1(new_n324));
  norb02aa1n02x5               g229(.a(new_n324), .b(new_n323), .out0(\s[6] ));
  nanb02aa1n02x5               g230(.a(new_n114), .b(new_n115), .out0(new_n326));
  aoib12aa1n06x5               g231(.a(new_n326), .b(new_n120), .c(new_n323), .out0(new_n327));
  nano22aa1n02x4               g232(.a(new_n323), .b(new_n326), .c(new_n120), .out0(new_n328));
  norp02aa1n02x5               g233(.a(new_n327), .b(new_n328), .o1(\s[7] ));
  obai22aa1n02x7               g234(.a(new_n113), .b(new_n112), .c(new_n327), .d(new_n114), .out0(new_n330));
  oaib12aa1n02x5               g235(.a(new_n330), .b(new_n327), .c(new_n118), .out0(\s[8] ));
  aoi112aa1n02x5               g236(.a(new_n121), .b(new_n123), .c(new_n109), .d(new_n117), .o1(new_n332));
  norb02aa1n02x5               g237(.a(new_n124), .b(new_n332), .out0(\s[9] ));
endmodule


