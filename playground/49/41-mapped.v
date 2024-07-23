// Benchmark "adder" written by ABC on Thu Jul 18 14:56:24 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n184, new_n185, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n326, new_n327,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  and002aa1n03x5               g002(.a(\b[0] ), .b(\a[1] ), .o(new_n98));
  tech160nm_fioaoi03aa1n02p5x5 g003(.a(\a[2] ), .b(\b[1] ), .c(new_n98), .o1(new_n99));
  xorc02aa1n02x5               g004(.a(\a[4] ), .b(\b[3] ), .out0(new_n100));
  nor002aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  norb02aa1n02x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  nanp03aa1n03x5               g008(.a(new_n99), .b(new_n100), .c(new_n103), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\a[4] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(\b[3] ), .o1(new_n106));
  oaoi03aa1n02x5               g011(.a(new_n105), .b(new_n106), .c(new_n101), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand42aa1n04x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor042aa1n03x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand42aa1n03x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nano23aa1n02x4               g016(.a(new_n108), .b(new_n110), .c(new_n111), .d(new_n109), .out0(new_n112));
  tech160nm_fixorc02aa1n02p5x5 g017(.a(\a[5] ), .b(\b[4] ), .out0(new_n113));
  xorc02aa1n02x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  nanp03aa1n02x5               g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  oai022aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  nano22aa1n03x5               g022(.a(new_n110), .b(new_n109), .c(new_n111), .out0(new_n118));
  oai022aa1n02x5               g023(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  aoai13aa1n06x5               g024(.a(new_n116), .b(new_n119), .c(new_n118), .d(new_n117), .o1(new_n120));
  aoai13aa1n12x5               g025(.a(new_n120), .b(new_n115), .c(new_n104), .d(new_n107), .o1(new_n121));
  xnrc02aa1n02x5               g026(.a(\b[8] ), .b(\a[9] ), .out0(new_n122));
  nanb02aa1n02x5               g027(.a(new_n122), .b(new_n121), .out0(new_n123));
  xnrc02aa1n02x5               g028(.a(\b[9] ), .b(\a[10] ), .out0(new_n124));
  xobna2aa1n03x5               g029(.a(new_n124), .b(new_n123), .c(new_n97), .out0(\s[10] ));
  nanp02aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  nor022aa1n08x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nanp02aa1n04x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n127), .b(new_n128), .out0(new_n129));
  oai022aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n130), .b(new_n123), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n129), .b(new_n131), .c(new_n126), .out0(\s[11] ));
  nano22aa1n02x4               g037(.a(new_n127), .b(new_n126), .c(new_n128), .out0(new_n133));
  tech160nm_fiaoi012aa1n05x5   g038(.a(new_n127), .b(new_n131), .c(new_n133), .o1(new_n134));
  xnrb03aa1n03x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n04x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nanp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nano23aa1n06x5               g042(.a(new_n127), .b(new_n136), .c(new_n137), .d(new_n128), .out0(new_n138));
  nona22aa1n06x5               g043(.a(new_n138), .b(new_n124), .c(new_n122), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(new_n121), .b(new_n140), .o1(new_n141));
  nona23aa1n06x5               g046(.a(new_n137), .b(new_n128), .c(new_n127), .d(new_n136), .out0(new_n142));
  nanp02aa1n02x5               g047(.a(new_n130), .b(new_n126), .o1(new_n143));
  oa0012aa1n02x5               g048(.a(new_n137), .b(new_n136), .c(new_n127), .o(new_n144));
  oabi12aa1n12x5               g049(.a(new_n144), .b(new_n142), .c(new_n143), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n141), .b(new_n146), .o1(new_n147));
  xorb03aa1n02x5               g052(.a(new_n147), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n02x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n150), .o1(new_n151));
  xnrb03aa1n02x5               g056(.a(new_n151), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  nano23aa1n06x5               g059(.a(new_n149), .b(new_n153), .c(new_n154), .d(new_n150), .out0(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n145), .c(new_n121), .d(new_n140), .o1(new_n156));
  aoi012aa1n09x5               g061(.a(new_n153), .b(new_n149), .c(new_n154), .o1(new_n157));
  nor042aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  nanb02aa1n02x5               g064(.a(new_n158), .b(new_n159), .out0(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n156), .c(new_n157), .out0(\s[15] ));
  nanp02aa1n02x5               g067(.a(new_n156), .b(new_n157), .o1(new_n163));
  nor002aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  nand42aa1n02x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nanb02aa1n02x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  aoai13aa1n02x5               g071(.a(new_n166), .b(new_n158), .c(new_n163), .d(new_n161), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n158), .b(new_n166), .c(new_n163), .d(new_n159), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(\s[16] ));
  inv000aa1d42x5               g074(.a(\a[17] ), .o1(new_n170));
  nano23aa1n06x5               g075(.a(new_n158), .b(new_n164), .c(new_n165), .d(new_n159), .out0(new_n171));
  nano22aa1d15x5               g076(.a(new_n139), .b(new_n155), .c(new_n171), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n171), .o1(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n174));
  aoai13aa1n04x5               g079(.a(new_n155), .b(new_n144), .c(new_n138), .d(new_n174), .o1(new_n175));
  tech160nm_fiaoi012aa1n04x5   g080(.a(new_n173), .b(new_n175), .c(new_n157), .o1(new_n176));
  oa0012aa1n18x5               g081(.a(new_n165), .b(new_n164), .c(new_n158), .o(new_n177));
  aoi112aa1n09x5               g082(.a(new_n176), .b(new_n177), .c(new_n121), .d(new_n172), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(new_n170), .out0(\s[17] ));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(new_n180), .b(new_n170), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n177), .o1(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n173), .c(new_n175), .d(new_n157), .o1(new_n183));
  xorc02aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .out0(new_n184));
  aoai13aa1n02x5               g089(.a(new_n184), .b(new_n183), .c(new_n121), .d(new_n172), .o1(new_n185));
  norp02aa1n02x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  xobna2aa1n03x5               g093(.a(new_n188), .b(new_n185), .c(new_n181), .out0(\s[18] ));
  inv000aa1d42x5               g094(.a(\a[18] ), .o1(new_n190));
  xroi22aa1d06x4               g095(.a(new_n170), .b(\b[16] ), .c(new_n190), .d(\b[17] ), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  aoi013aa1n09x5               g097(.a(new_n186), .b(new_n187), .c(new_n170), .d(new_n180), .o1(new_n193));
  tech160nm_fioai012aa1n05x5   g098(.a(new_n193), .b(new_n178), .c(new_n192), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n08x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand42aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nor022aa1n06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nand42aa1n02x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  aoai13aa1n02x5               g107(.a(new_n202), .b(new_n197), .c(new_n194), .d(new_n199), .o1(new_n203));
  nanp02aa1n06x5               g108(.a(new_n121), .b(new_n172), .o1(new_n204));
  inv000aa1d42x5               g109(.a(new_n157), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n171), .b(new_n205), .c(new_n145), .d(new_n155), .o1(new_n206));
  nand23aa1n04x5               g111(.a(new_n204), .b(new_n206), .c(new_n182), .o1(new_n207));
  oaoi03aa1n02x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n181), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n199), .b(new_n208), .c(new_n207), .d(new_n191), .o1(new_n209));
  nona22aa1n02x4               g114(.a(new_n209), .b(new_n202), .c(new_n197), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n203), .b(new_n210), .o1(\s[20] ));
  nona23aa1n06x5               g116(.a(new_n201), .b(new_n198), .c(new_n197), .d(new_n200), .out0(new_n212));
  norb03aa1n03x5               g117(.a(new_n184), .b(new_n212), .c(new_n188), .out0(new_n213));
  inv040aa1n03x5               g118(.a(new_n213), .o1(new_n214));
  oa0012aa1n02x5               g119(.a(new_n201), .b(new_n200), .c(new_n197), .o(new_n215));
  oabi12aa1n18x5               g120(.a(new_n215), .b(new_n193), .c(new_n212), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  tech160nm_fioai012aa1n05x5   g122(.a(new_n217), .b(new_n178), .c(new_n214), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  xnrc02aa1n03x5               g127(.a(\b[21] ), .b(\a[22] ), .out0(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n220), .c(new_n218), .d(new_n222), .o1(new_n224));
  aoai13aa1n02x7               g129(.a(new_n222), .b(new_n216), .c(new_n207), .d(new_n213), .o1(new_n225));
  nona22aa1n02x4               g130(.a(new_n225), .b(new_n223), .c(new_n220), .out0(new_n226));
  nanp02aa1n02x5               g131(.a(new_n224), .b(new_n226), .o1(\s[22] ));
  nano23aa1n06x5               g132(.a(new_n197), .b(new_n200), .c(new_n201), .d(new_n198), .out0(new_n228));
  nor042aa1n06x5               g133(.a(new_n223), .b(new_n221), .o1(new_n229));
  nano22aa1n03x7               g134(.a(new_n192), .b(new_n229), .c(new_n228), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n230), .b(new_n183), .c(new_n121), .d(new_n172), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n230), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\a[22] ), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\b[21] ), .o1(new_n234));
  oao003aa1n02x5               g139(.a(new_n233), .b(new_n234), .c(new_n220), .carry(new_n235));
  aoi012aa1d24x5               g140(.a(new_n235), .b(new_n216), .c(new_n229), .o1(new_n236));
  tech160nm_fioai012aa1n04x5   g141(.a(new_n236), .b(new_n178), .c(new_n232), .o1(new_n237));
  xorc02aa1n06x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  aoi112aa1n02x5               g143(.a(new_n238), .b(new_n235), .c(new_n216), .d(new_n229), .o1(new_n239));
  aoi022aa1n02x5               g144(.a(new_n237), .b(new_n238), .c(new_n231), .d(new_n239), .o1(\s[23] ));
  norp02aa1n02x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  norp02aa1n02x5               g146(.a(\b[23] ), .b(\a[24] ), .o1(new_n242));
  nanp02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  nanb02aa1n06x5               g148(.a(new_n242), .b(new_n243), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n241), .c(new_n237), .d(new_n238), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n236), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n238), .b(new_n246), .c(new_n207), .d(new_n230), .o1(new_n247));
  nona22aa1n02x4               g152(.a(new_n247), .b(new_n244), .c(new_n241), .out0(new_n248));
  nanp02aa1n02x5               g153(.a(new_n245), .b(new_n248), .o1(\s[24] ));
  norb02aa1n03x5               g154(.a(new_n238), .b(new_n244), .out0(new_n250));
  inv040aa1n03x5               g155(.a(new_n250), .o1(new_n251));
  nano32aa1n03x7               g156(.a(new_n251), .b(new_n191), .c(new_n229), .d(new_n228), .out0(new_n252));
  inv000aa1n02x5               g157(.a(new_n252), .o1(new_n253));
  aoai13aa1n09x5               g158(.a(new_n229), .b(new_n215), .c(new_n228), .d(new_n208), .o1(new_n254));
  inv040aa1n02x5               g159(.a(new_n235), .o1(new_n255));
  oai012aa1n02x5               g160(.a(new_n243), .b(new_n242), .c(new_n241), .o1(new_n256));
  aoai13aa1n12x5               g161(.a(new_n256), .b(new_n251), .c(new_n254), .d(new_n255), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  tech160nm_fioai012aa1n05x5   g163(.a(new_n258), .b(new_n178), .c(new_n253), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n06x5               g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  xnrc02aa1n02x5               g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  aoai13aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n262), .b(new_n257), .c(new_n207), .d(new_n252), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n265), .b(new_n263), .c(new_n261), .out0(new_n266));
  nanp02aa1n02x5               g171(.a(new_n264), .b(new_n266), .o1(\s[26] ));
  norb02aa1n03x5               g172(.a(new_n262), .b(new_n263), .out0(new_n268));
  nano23aa1n03x7               g173(.a(new_n214), .b(new_n251), .c(new_n268), .d(new_n229), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n183), .c(new_n121), .d(new_n172), .o1(new_n270));
  nanp02aa1n02x5               g175(.a(\b[25] ), .b(\a[26] ), .o1(new_n271));
  oai022aa1n02x5               g176(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n272));
  aoi022aa1n12x5               g177(.a(new_n257), .b(new_n268), .c(new_n271), .d(new_n272), .o1(new_n273));
  xorc02aa1n12x5               g178(.a(\a[27] ), .b(\b[26] ), .out0(new_n274));
  xnbna2aa1n03x5               g179(.a(new_n274), .b(new_n270), .c(new_n273), .out0(\s[27] ));
  inv000aa1n02x5               g180(.a(new_n269), .o1(new_n276));
  oai012aa1n04x7               g181(.a(new_n273), .b(new_n178), .c(new_n276), .o1(new_n277));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  norp02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .o1(new_n279));
  nanp02aa1n04x5               g184(.a(\b[27] ), .b(\a[28] ), .o1(new_n280));
  nanb02aa1n06x5               g185(.a(new_n279), .b(new_n280), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n278), .c(new_n277), .d(new_n274), .o1(new_n282));
  aoai13aa1n09x5               g187(.a(new_n250), .b(new_n235), .c(new_n216), .d(new_n229), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n268), .o1(new_n284));
  nanp02aa1n02x5               g189(.a(new_n272), .b(new_n271), .o1(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n284), .c(new_n283), .d(new_n256), .o1(new_n286));
  aoai13aa1n02x5               g191(.a(new_n274), .b(new_n286), .c(new_n207), .d(new_n269), .o1(new_n287));
  nona22aa1n02x4               g192(.a(new_n287), .b(new_n281), .c(new_n278), .out0(new_n288));
  nanp02aa1n02x5               g193(.a(new_n282), .b(new_n288), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n274), .b(new_n281), .out0(new_n290));
  aoai13aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n207), .d(new_n269), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n290), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n279), .b(new_n278), .c(new_n280), .o1(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n292), .c(new_n270), .d(new_n273), .o1(new_n294));
  tech160nm_fixorc02aa1n02p5x5 g199(.a(\a[29] ), .b(\b[28] ), .out0(new_n295));
  norb02aa1n02x5               g200(.a(new_n293), .b(new_n295), .out0(new_n296));
  aoi022aa1n02x7               g201(.a(new_n294), .b(new_n295), .c(new_n291), .d(new_n296), .o1(\s[29] ));
  xnrb03aa1n02x5               g202(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g203(.a(new_n281), .b(new_n274), .c(new_n295), .out0(new_n299));
  aoai13aa1n02x5               g204(.a(new_n299), .b(new_n286), .c(new_n207), .d(new_n269), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n299), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  aoai13aa1n02x5               g207(.a(new_n302), .b(new_n301), .c(new_n270), .d(new_n273), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .out0(new_n304));
  norb02aa1n02x5               g209(.a(new_n302), .b(new_n304), .out0(new_n305));
  aoi022aa1n02x7               g210(.a(new_n303), .b(new_n304), .c(new_n300), .d(new_n305), .o1(\s[30] ));
  nanp03aa1n02x5               g211(.a(new_n290), .b(new_n295), .c(new_n304), .o1(new_n307));
  nanb02aa1n02x5               g212(.a(new_n307), .b(new_n277), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n309));
  aoai13aa1n02x5               g214(.a(new_n309), .b(new_n307), .c(new_n270), .d(new_n273), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[31] ), .b(\b[30] ), .out0(new_n311));
  and002aa1n02x5               g216(.a(\b[29] ), .b(\a[30] ), .o(new_n312));
  oabi12aa1n02x5               g217(.a(new_n311), .b(\a[30] ), .c(\b[29] ), .out0(new_n313));
  oab012aa1n02x4               g218(.a(new_n313), .b(new_n302), .c(new_n312), .out0(new_n314));
  aoi022aa1n02x5               g219(.a(new_n310), .b(new_n311), .c(new_n308), .d(new_n314), .o1(\s[31] ));
  xorb03aa1n02x5               g220(.a(new_n99), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g221(.a(new_n102), .b(new_n99), .c(new_n101), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(new_n105), .out0(\s[4] ));
  xnbna2aa1n03x5               g223(.a(new_n113), .b(new_n104), .c(new_n107), .out0(\s[5] ));
  norb02aa1n02x5               g224(.a(new_n109), .b(new_n108), .out0(new_n320));
  orn002aa1n02x5               g225(.a(\a[5] ), .b(\b[4] ), .o(new_n321));
  nanp02aa1n02x5               g226(.a(new_n104), .b(new_n107), .o1(new_n322));
  nanp02aa1n02x5               g227(.a(new_n322), .b(new_n113), .o1(new_n323));
  nanb03aa1n02x5               g228(.a(new_n117), .b(new_n323), .c(new_n109), .out0(new_n324));
  aoai13aa1n02x5               g229(.a(new_n324), .b(new_n320), .c(new_n323), .d(new_n321), .o1(\s[6] ));
  nanb02aa1n02x5               g230(.a(new_n110), .b(new_n111), .out0(new_n326));
  aoai13aa1n02x5               g231(.a(new_n109), .b(new_n117), .c(new_n322), .d(new_n113), .o1(new_n327));
  aoi022aa1n02x5               g232(.a(new_n324), .b(new_n118), .c(new_n326), .d(new_n327), .o1(\s[7] ));
  aoi012aa1n02x5               g233(.a(new_n110), .b(new_n324), .c(new_n118), .o1(new_n329));
  xnrc02aa1n02x5               g234(.a(new_n329), .b(new_n114), .out0(\s[8] ));
  xorb03aa1n02x5               g235(.a(new_n121), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


