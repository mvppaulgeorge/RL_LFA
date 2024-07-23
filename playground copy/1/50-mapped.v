// Benchmark "adder" written by ABC on Wed Jul 10 16:42:40 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n140, new_n142, new_n143, new_n144, new_n145, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n322, new_n323, new_n325;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[4] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\a[3] ), .clkout(new_n101));
  160nm_ficinv00aa1n08x5       g006(.clk(\b[2] ), .clkout(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n101), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(new_n103), .b(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  aboi22aa1n03x5               g014(.a(\b[3] ), .b(new_n100), .c(new_n101), .d(new_n102), .out0(new_n110));
  oai012aa1n02x5               g015(.a(new_n110), .b(new_n109), .c(new_n105), .o1(new_n111));
  oaib12aa1n02x5               g016(.a(new_n111), .b(new_n100), .c(\b[3] ), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[6] ), .b(\b[5] ), .out0(new_n113));
  norp02aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanb02aa1n02x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nona23aa1n02x4               g023(.a(new_n113), .b(new_n118), .c(new_n117), .d(new_n116), .out0(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  oai022aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nano23aa1n02x4               g027(.a(new_n117), .b(new_n116), .c(new_n122), .d(new_n121), .out0(new_n123));
  norp03aa1n02x5               g028(.a(new_n123), .b(new_n120), .c(new_n114), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n124), .b(new_n112), .c(new_n119), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n98), .b(new_n99), .c(new_n125), .d(new_n126), .o1(new_n127));
  and002aa1n02x5               g032(.a(\b[3] ), .b(\a[4] ), .o(new_n128));
  oaoi13aa1n02x5               g033(.a(new_n128), .b(new_n110), .c(new_n109), .d(new_n105), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n115), .b(new_n114), .out0(new_n130));
  xorc02aa1n02x5               g035(.a(\a[7] ), .b(\b[6] ), .out0(new_n131));
  nanp02aa1n02x5               g036(.a(new_n131), .b(new_n130), .o1(new_n132));
  nano22aa1n02x4               g037(.a(new_n132), .b(new_n113), .c(new_n118), .out0(new_n133));
  160nm_ficinv00aa1n08x5       g038(.clk(new_n121), .clkout(new_n134));
  norp02aa1n02x5               g039(.a(\b[5] ), .b(\a[6] ), .o1(new_n135));
  oab012aa1n02x4               g040(.a(new_n135), .b(\a[5] ), .c(\b[4] ), .out0(new_n136));
  nona23aa1n02x4               g041(.a(new_n131), .b(new_n130), .c(new_n136), .d(new_n134), .out0(new_n137));
  nona22aa1n02x4               g042(.a(new_n137), .b(new_n120), .c(new_n114), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n126), .b(new_n138), .c(new_n129), .d(new_n133), .o1(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n99), .c(new_n98), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n127), .b(new_n140), .o1(\s[10] ));
  nanp02aa1n02x5               g046(.a(\b[9] ), .b(\a[10] ), .o1(new_n142));
  norp02aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(\b[10] ), .b(\a[11] ), .o1(new_n144));
  nanb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  xnbna2aa1n03x5               g050(.a(new_n145), .b(new_n140), .c(new_n142), .out0(\s[11] ));
  aoi013aa1n02x4               g051(.a(new_n143), .b(new_n140), .c(new_n142), .d(new_n144), .o1(new_n147));
  norp02aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  norb02aa1n02x5               g054(.a(new_n149), .b(new_n148), .out0(new_n150));
  xnrc02aa1n02x5               g055(.a(new_n147), .b(new_n150), .out0(\s[12] ));
  nano23aa1n02x4               g056(.a(new_n143), .b(new_n148), .c(new_n149), .d(new_n144), .out0(new_n152));
  and003aa1n02x5               g057(.a(new_n152), .b(new_n126), .c(new_n97), .o(new_n153));
  aoai13aa1n02x5               g058(.a(new_n153), .b(new_n138), .c(new_n129), .d(new_n133), .o1(new_n154));
  aoi112aa1n02x5               g059(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n155));
  nanb02aa1n02x5               g060(.a(new_n148), .b(new_n149), .out0(new_n156));
  oai022aa1n02x5               g061(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n157));
  nano23aa1n02x4               g062(.a(new_n156), .b(new_n145), .c(new_n157), .d(new_n142), .out0(new_n158));
  norp03aa1n02x5               g063(.a(new_n158), .b(new_n155), .c(new_n148), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n154), .b(new_n159), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n160), .c(new_n163), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nano23aa1n02x4               g072(.a(new_n162), .b(new_n166), .c(new_n167), .d(new_n163), .out0(new_n168));
  160nm_ficinv00aa1n08x5       g073(.clk(new_n168), .clkout(new_n169));
  oai012aa1n02x5               g074(.a(new_n167), .b(new_n166), .c(new_n162), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n154), .d(new_n159), .o1(new_n171));
  xorb03aa1n02x5               g076(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  nanp02aa1n02x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  norp02aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aoi112aa1n02x5               g083(.a(new_n178), .b(new_n173), .c(new_n171), .d(new_n175), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n178), .b(new_n173), .c(new_n171), .d(new_n174), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(\s[16] ));
  nanp03aa1n02x5               g086(.a(new_n168), .b(new_n175), .c(new_n178), .o1(new_n182));
  nano32aa1n02x4               g087(.a(new_n182), .b(new_n152), .c(new_n126), .d(new_n97), .out0(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n138), .c(new_n129), .d(new_n133), .o1(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(new_n142), .clkout(new_n185));
  norb02aa1n02x5               g090(.a(new_n144), .b(new_n143), .out0(new_n186));
  oab012aa1n02x4               g091(.a(new_n99), .b(\a[10] ), .c(\b[9] ), .out0(new_n187));
  nona23aa1n02x4               g092(.a(new_n150), .b(new_n186), .c(new_n187), .d(new_n185), .out0(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(new_n155), .c(new_n148), .out0(new_n189));
  160nm_ficinv00aa1n08x5       g094(.clk(new_n182), .clkout(new_n190));
  nona23aa1n02x4               g095(.a(new_n177), .b(new_n174), .c(new_n173), .d(new_n176), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(new_n173), .b(new_n177), .o1(new_n192));
  oai122aa1n02x7               g097(.a(new_n192), .b(new_n191), .c(new_n170), .d(\b[15] ), .e(\a[16] ), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n193), .b(new_n189), .c(new_n190), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  xobna2aa1n03x5               g102(.a(new_n197), .b(new_n184), .c(new_n194), .out0(\s[17] ));
  nanb03aa1n02x5               g103(.a(new_n197), .b(new_n184), .c(new_n194), .out0(new_n199));
  norp02aa1n02x5               g104(.a(\b[17] ), .b(\a[18] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  norb02aa1n02x5               g106(.a(new_n201), .b(new_n200), .out0(new_n202));
  xobna2aa1n03x5               g107(.a(new_n202), .b(new_n199), .c(new_n196), .out0(\s[18] ));
  nano23aa1n02x4               g108(.a(new_n195), .b(new_n200), .c(new_n201), .d(new_n196), .out0(new_n204));
  160nm_ficinv00aa1n08x5       g109(.clk(new_n204), .clkout(new_n205));
  oai012aa1n02x5               g110(.a(new_n201), .b(new_n200), .c(new_n195), .o1(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n205), .c(new_n184), .d(new_n194), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norp02aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanp02aa1n02x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n210), .b(new_n214), .c(new_n207), .d(new_n211), .o1(new_n215));
  aoai13aa1n02x5               g120(.a(new_n214), .b(new_n210), .c(new_n207), .d(new_n211), .o1(new_n216));
  norb02aa1n02x5               g121(.a(new_n216), .b(new_n215), .out0(\s[20] ));
  nano23aa1n02x4               g122(.a(new_n210), .b(new_n212), .c(new_n213), .d(new_n211), .out0(new_n218));
  nanp02aa1n02x5               g123(.a(new_n218), .b(new_n204), .o1(new_n219));
  nona23aa1n02x4               g124(.a(new_n213), .b(new_n211), .c(new_n210), .d(new_n212), .out0(new_n220));
  aoi012aa1n02x5               g125(.a(new_n212), .b(new_n210), .c(new_n213), .o1(new_n221));
  oai012aa1n02x5               g126(.a(new_n221), .b(new_n220), .c(new_n206), .o1(new_n222));
  160nm_ficinv00aa1n08x5       g127(.clk(new_n222), .clkout(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n219), .c(new_n184), .d(new_n194), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g130(.a(\b[20] ), .b(\a[21] ), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[21] ), .b(\b[20] ), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[22] ), .b(\b[21] ), .out0(new_n228));
  aoi112aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g136(.clk(\a[21] ), .clkout(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(\a[22] ), .clkout(new_n233));
  xroi22aa1d04x5               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  nanp03aa1n02x5               g139(.a(new_n234), .b(new_n204), .c(new_n218), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n206), .clkout(new_n236));
  160nm_ficinv00aa1n08x5       g141(.clk(new_n221), .clkout(new_n237));
  aoai13aa1n02x5               g142(.a(new_n234), .b(new_n237), .c(new_n218), .d(new_n236), .o1(new_n238));
  160nm_ficinv00aa1n08x5       g143(.clk(\b[21] ), .clkout(new_n239));
  oaoi03aa1n02x5               g144(.a(new_n233), .b(new_n239), .c(new_n226), .o1(new_n240));
  nanp02aa1n02x5               g145(.a(new_n238), .b(new_n240), .o1(new_n241));
  160nm_ficinv00aa1n08x5       g146(.clk(new_n241), .clkout(new_n242));
  aoai13aa1n02x5               g147(.a(new_n242), .b(new_n235), .c(new_n184), .d(new_n194), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  xorc02aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  xorc02aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  aoi112aa1n02x5               g152(.a(new_n245), .b(new_n247), .c(new_n243), .d(new_n246), .o1(new_n248));
  aoai13aa1n02x5               g153(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n246), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(\s[24] ));
  and002aa1n02x5               g155(.a(new_n247), .b(new_n246), .o(new_n251));
  nanb03aa1n02x5               g156(.a(new_n219), .b(new_n251), .c(new_n234), .out0(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n251), .clkout(new_n253));
  orn002aa1n02x5               g158(.a(\a[23] ), .b(\b[22] ), .o(new_n254));
  oao003aa1n02x5               g159(.a(\a[24] ), .b(\b[23] ), .c(new_n254), .carry(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n238), .d(new_n240), .o1(new_n256));
  160nm_ficinv00aa1n08x5       g161(.clk(new_n256), .clkout(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n252), .c(new_n184), .d(new_n194), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .out0(new_n261));
  xorc02aa1n02x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  aoi112aa1n02x5               g167(.a(new_n260), .b(new_n262), .c(new_n258), .d(new_n261), .o1(new_n263));
  aoai13aa1n02x5               g168(.a(new_n262), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n264), .b(new_n263), .out0(\s[26] ));
  oabi12aa1n02x5               g170(.a(new_n193), .b(new_n159), .c(new_n182), .out0(new_n266));
  and002aa1n02x5               g171(.a(new_n262), .b(new_n261), .o(new_n267));
  nano22aa1n02x4               g172(.a(new_n235), .b(new_n251), .c(new_n267), .out0(new_n268));
  aoai13aa1n02x5               g173(.a(new_n268), .b(new_n266), .c(new_n125), .d(new_n183), .o1(new_n269));
  orn002aa1n02x5               g174(.a(\a[25] ), .b(\b[24] ), .o(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n270), .carry(new_n271));
  aobi12aa1n02x5               g176(.a(new_n271), .b(new_n256), .c(new_n267), .out0(new_n272));
  norp02aa1n02x5               g177(.a(\b[26] ), .b(\a[27] ), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n274), .b(new_n273), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n272), .c(new_n269), .out0(\s[27] ));
  160nm_ficinv00aa1n08x5       g181(.clk(new_n273), .clkout(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  160nm_ficinv00aa1n08x5       g183(.clk(new_n268), .clkout(new_n279));
  aoi012aa1n02x5               g184(.a(new_n279), .b(new_n184), .c(new_n194), .o1(new_n280));
  160nm_ficinv00aa1n08x5       g185(.clk(new_n240), .clkout(new_n281));
  aoai13aa1n02x5               g186(.a(new_n251), .b(new_n281), .c(new_n222), .d(new_n234), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n267), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n271), .b(new_n283), .c(new_n282), .d(new_n255), .o1(new_n284));
  oai012aa1n02x5               g189(.a(new_n274), .b(new_n284), .c(new_n280), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n278), .b(new_n285), .c(new_n277), .o1(new_n286));
  aobi12aa1n02x5               g191(.a(new_n274), .b(new_n272), .c(new_n269), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n277), .c(new_n278), .out0(new_n288));
  norp02aa1n02x5               g193(.a(new_n286), .b(new_n288), .o1(\s[28] ));
  nano22aa1n02x4               g194(.a(new_n278), .b(new_n277), .c(new_n274), .out0(new_n290));
  oai012aa1n02x5               g195(.a(new_n290), .b(new_n284), .c(new_n280), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n277), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  aoi012aa1n02x5               g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n02x5               g199(.a(new_n290), .b(new_n272), .c(new_n269), .out0(new_n295));
  nano22aa1n02x4               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n02x5               g201(.a(new_n294), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n275), .b(new_n293), .c(new_n278), .out0(new_n299));
  oai012aa1n02x5               g204(.a(new_n299), .b(new_n284), .c(new_n280), .o1(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  aobi12aa1n02x5               g208(.a(new_n299), .b(new_n272), .c(new_n269), .out0(new_n304));
  nano22aa1n02x4               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[30] ));
  norb03aa1n02x5               g211(.a(new_n290), .b(new_n302), .c(new_n293), .out0(new_n307));
  aobi12aa1n02x5               g212(.a(new_n307), .b(new_n272), .c(new_n269), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nano22aa1n02x4               g215(.a(new_n308), .b(new_n309), .c(new_n310), .out0(new_n311));
  oai012aa1n02x5               g216(.a(new_n307), .b(new_n284), .c(new_n280), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n310), .b(new_n312), .c(new_n309), .o1(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnbna2aa1n03x5               g219(.a(new_n109), .b(new_n103), .c(new_n104), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n129), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g223(.a(\a[5] ), .b(\b[4] ), .c(new_n112), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_ficinv00aa1n08x5       g225(.clk(new_n113), .clkout(new_n321));
  oai112aa1n02x5               g226(.a(new_n121), .b(new_n131), .c(new_n319), .d(new_n321), .o1(new_n322));
  oaoi13aa1n02x5               g227(.a(new_n131), .b(new_n121), .c(new_n319), .d(new_n321), .o1(new_n323));
  norb02aa1n02x5               g228(.a(new_n322), .b(new_n323), .out0(\s[7] ));
  orn002aa1n02x5               g229(.a(\a[7] ), .b(\b[6] ), .o(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n130), .b(new_n322), .c(new_n325), .out0(\s[8] ));
  xorb03aa1n02x5               g231(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


