// Benchmark "adder" written by ABC on Thu Jul 11 13:07:41 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n189, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n235, new_n236, new_n237,
    new_n238, new_n239, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n315, new_n317, new_n319;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  160nm_ficinv00aa1n08x5       g001(.clk(\a[10] ), .clkout(new_n97));
  norp02aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\a[2] ), .clkout(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\b[1] ), .clkout(new_n100));
  nanp02aa1n02x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oao003aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano23aa1n02x4               g011(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n107));
  nanp02aa1n02x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  aoi012aa1n02x5               g013(.a(new_n103), .b(new_n105), .c(new_n104), .o1(new_n109));
  norp02aa1n02x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n02x4               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  xorc02aa1n02x5               g019(.a(\a[6] ), .b(\b[5] ), .out0(new_n115));
  xorc02aa1n02x5               g020(.a(\a[5] ), .b(\b[4] ), .out0(new_n116));
  nanp03aa1n02x5               g021(.a(new_n114), .b(new_n115), .c(new_n116), .o1(new_n117));
  160nm_ficinv00aa1n08x5       g022(.clk(\b[4] ), .clkout(new_n118));
  nanb02aa1n02x5               g023(.a(\a[5] ), .b(new_n118), .out0(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[6] ), .b(\b[5] ), .c(new_n119), .o1(new_n120));
  oai012aa1n02x5               g025(.a(new_n111), .b(new_n112), .c(new_n110), .o1(new_n121));
  aobi12aa1n02x5               g026(.a(new_n121), .b(new_n114), .c(new_n120), .out0(new_n122));
  aoai13aa1n02x5               g027(.a(new_n122), .b(new_n117), .c(new_n108), .d(new_n109), .o1(new_n123));
  xnrc02aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  aoib12aa1n02x5               g029(.a(new_n98), .b(new_n123), .c(new_n124), .out0(new_n125));
  xorb03aa1n02x5               g030(.a(new_n125), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .out0(new_n127));
  norp02aa1n02x5               g032(.a(new_n124), .b(new_n127), .o1(new_n128));
  160nm_ficinv00aa1n08x5       g033(.clk(\b[9] ), .clkout(new_n129));
  oao003aa1n02x5               g034(.a(new_n97), .b(new_n129), .c(new_n98), .carry(new_n130));
  norp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n130), .c(new_n123), .d(new_n128), .o1(new_n134));
  aoi112aa1n02x5               g039(.a(new_n133), .b(new_n130), .c(new_n123), .d(new_n128), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g041(.clk(new_n131), .clkout(new_n137));
  norp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n134), .c(new_n137), .out0(\s[12] ));
  nano23aa1n02x4               g046(.a(new_n131), .b(new_n138), .c(new_n139), .d(new_n132), .out0(new_n142));
  oai012aa1n02x5               g047(.a(new_n139), .b(new_n138), .c(new_n131), .o1(new_n143));
  aobi12aa1n02x5               g048(.a(new_n143), .b(new_n142), .c(new_n130), .out0(new_n144));
  oaoi03aa1n02x5               g049(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n145));
  nona23aa1n02x4               g050(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n146));
  oai012aa1n02x5               g051(.a(new_n109), .b(new_n146), .c(new_n145), .o1(new_n147));
  nona23aa1n02x4               g052(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n148));
  nano22aa1n02x4               g053(.a(new_n148), .b(new_n115), .c(new_n116), .out0(new_n149));
  oaib12aa1n02x5               g054(.a(new_n121), .b(new_n148), .c(new_n120), .out0(new_n150));
  nona23aa1n02x4               g055(.a(new_n139), .b(new_n132), .c(new_n131), .d(new_n138), .out0(new_n151));
  norp03aa1n02x5               g056(.a(new_n151), .b(new_n124), .c(new_n127), .o1(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n150), .c(new_n149), .d(new_n147), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n144), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanp02aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nano23aa1n02x4               g066(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n162));
  160nm_ficinv00aa1n08x5       g067(.clk(new_n162), .clkout(new_n163));
  aoi012aa1n02x5               g068(.a(new_n160), .b(new_n156), .c(new_n161), .o1(new_n164));
  aoai13aa1n02x5               g069(.a(new_n164), .b(new_n163), .c(new_n153), .d(new_n144), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  xorc02aa1n02x5               g072(.a(\a[15] ), .b(\b[14] ), .out0(new_n168));
  xorc02aa1n02x5               g073(.a(\a[16] ), .b(\b[15] ), .out0(new_n169));
  aoi112aa1n02x5               g074(.a(new_n169), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(\s[16] ));
  nanp03aa1n02x5               g077(.a(new_n162), .b(new_n168), .c(new_n169), .o1(new_n173));
  nano22aa1n02x4               g078(.a(new_n173), .b(new_n128), .c(new_n142), .out0(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n150), .c(new_n147), .d(new_n149), .o1(new_n175));
  oaoi03aa1n02x5               g080(.a(new_n97), .b(new_n129), .c(new_n98), .o1(new_n176));
  oai012aa1n02x5               g081(.a(new_n143), .b(new_n151), .c(new_n176), .o1(new_n177));
  xnrc02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .out0(new_n178));
  xnrc02aa1n02x5               g083(.a(\b[15] ), .b(\a[16] ), .out0(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(new_n167), .clkout(new_n180));
  oao003aa1n02x5               g085(.a(\a[16] ), .b(\b[15] ), .c(new_n180), .carry(new_n181));
  oai013aa1n02x4               g086(.a(new_n181), .b(new_n179), .c(new_n178), .d(new_n164), .o1(new_n182));
  aoib12aa1n02x5               g087(.a(new_n182), .b(new_n177), .c(new_n173), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n175), .b(new_n183), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g090(.clk(\a[18] ), .clkout(new_n186));
  160nm_ficinv00aa1n08x5       g091(.clk(\a[17] ), .clkout(new_n187));
  160nm_ficinv00aa1n08x5       g092(.clk(\b[16] ), .clkout(new_n188));
  oaoi03aa1n02x5               g093(.a(new_n187), .b(new_n188), .c(new_n184), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[17] ), .c(new_n186), .out0(\s[18] ));
  xroi22aa1d04x5               g095(.a(new_n187), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n191));
  nanp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nona22aa1n02x4               g097(.a(new_n192), .b(\b[16] ), .c(\a[17] ), .out0(new_n193));
  oaib12aa1n02x5               g098(.a(new_n193), .b(\b[17] ), .c(new_n186), .out0(new_n194));
  norp02aa1n02x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n197), .b(new_n194), .c(new_n184), .d(new_n191), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(new_n197), .b(new_n194), .c(new_n184), .d(new_n191), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  nanp02aa1n02x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  nona22aa1n02x4               g109(.a(new_n198), .b(new_n204), .c(new_n195), .out0(new_n205));
  orn002aa1n02x5               g110(.a(\a[19] ), .b(\b[18] ), .o(new_n206));
  aobi12aa1n02x5               g111(.a(new_n204), .b(new_n198), .c(new_n206), .out0(new_n207));
  norb02aa1n02x5               g112(.a(new_n205), .b(new_n207), .out0(\s[20] ));
  nano23aa1n02x4               g113(.a(new_n195), .b(new_n202), .c(new_n203), .d(new_n196), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n191), .b(new_n209), .o1(new_n210));
  norp02aa1n02x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  aoi013aa1n02x4               g116(.a(new_n211), .b(new_n192), .c(new_n187), .d(new_n188), .o1(new_n212));
  nona23aa1n02x4               g117(.a(new_n203), .b(new_n196), .c(new_n195), .d(new_n202), .out0(new_n213));
  oaoi03aa1n02x5               g118(.a(\a[20] ), .b(\b[19] ), .c(new_n206), .o1(new_n214));
  160nm_ficinv00aa1n08x5       g119(.clk(new_n214), .clkout(new_n215));
  oai012aa1n02x5               g120(.a(new_n215), .b(new_n213), .c(new_n212), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  aoai13aa1n02x5               g122(.a(new_n217), .b(new_n210), .c(new_n175), .d(new_n183), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xorc02aa1n02x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  xorc02aa1n02x5               g126(.a(\a[22] ), .b(\b[21] ), .out0(new_n222));
  aoi112aa1n02x5               g127(.a(new_n220), .b(new_n222), .c(new_n218), .d(new_n221), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n222), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g130(.clk(\a[21] ), .clkout(new_n226));
  160nm_ficinv00aa1n08x5       g131(.clk(\a[22] ), .clkout(new_n227));
  xroi22aa1d04x5               g132(.a(new_n226), .b(\b[20] ), .c(new_n227), .d(\b[21] ), .out0(new_n228));
  nanp03aa1n02x5               g133(.a(new_n228), .b(new_n191), .c(new_n209), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(\b[21] ), .clkout(new_n230));
  oao003aa1n02x5               g135(.a(new_n227), .b(new_n230), .c(new_n220), .carry(new_n231));
  aoi012aa1n02x5               g136(.a(new_n231), .b(new_n216), .c(new_n228), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n229), .c(new_n175), .d(new_n183), .o1(new_n233));
  xorb03aa1n02x5               g138(.a(new_n233), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g139(.a(\b[22] ), .b(\a[23] ), .o1(new_n235));
  xorc02aa1n02x5               g140(.a(\a[23] ), .b(\b[22] ), .out0(new_n236));
  xorc02aa1n02x5               g141(.a(\a[24] ), .b(\b[23] ), .out0(new_n237));
  aoi112aa1n02x5               g142(.a(new_n235), .b(new_n237), .c(new_n233), .d(new_n236), .o1(new_n238));
  aoai13aa1n02x5               g143(.a(new_n237), .b(new_n235), .c(new_n233), .d(new_n236), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n239), .b(new_n238), .out0(\s[24] ));
  oabi12aa1n02x5               g145(.a(new_n182), .b(new_n144), .c(new_n173), .out0(new_n241));
  and002aa1n02x5               g146(.a(new_n237), .b(new_n236), .o(new_n242));
  160nm_ficinv00aa1n08x5       g147(.clk(new_n242), .clkout(new_n243));
  nano32aa1n02x4               g148(.a(new_n243), .b(new_n228), .c(new_n191), .d(new_n209), .out0(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n241), .c(new_n123), .d(new_n174), .o1(new_n245));
  aoai13aa1n02x5               g150(.a(new_n228), .b(new_n214), .c(new_n209), .d(new_n194), .o1(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n231), .clkout(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(\a[24] ), .clkout(new_n248));
  160nm_ficinv00aa1n08x5       g153(.clk(\b[23] ), .clkout(new_n249));
  oao003aa1n02x5               g154(.a(new_n248), .b(new_n249), .c(new_n235), .carry(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n250), .clkout(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n243), .c(new_n246), .d(new_n247), .o1(new_n252));
  160nm_ficinv00aa1n08x5       g157(.clk(new_n252), .clkout(new_n253));
  xorc02aa1n02x5               g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n245), .c(new_n253), .out0(\s[25] ));
  nanp02aa1n02x5               g160(.a(new_n245), .b(new_n253), .o1(new_n256));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  norp02aa1n02x5               g162(.a(\b[25] ), .b(\a[26] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  aoi112aa1n02x5               g165(.a(new_n257), .b(new_n260), .c(new_n256), .d(new_n254), .o1(new_n261));
  aoai13aa1n02x5               g166(.a(new_n260), .b(new_n257), .c(new_n256), .d(new_n254), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(\s[26] ));
  and002aa1n02x5               g168(.a(new_n254), .b(new_n260), .o(new_n264));
  nano22aa1n02x4               g169(.a(new_n229), .b(new_n242), .c(new_n264), .out0(new_n265));
  aoai13aa1n02x5               g170(.a(new_n265), .b(new_n241), .c(new_n123), .d(new_n174), .o1(new_n266));
  oai012aa1n02x5               g171(.a(new_n259), .b(new_n258), .c(new_n257), .o1(new_n267));
  aobi12aa1n02x5               g172(.a(new_n267), .b(new_n252), .c(new_n264), .out0(new_n268));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  norb02aa1n02x5               g175(.a(new_n270), .b(new_n269), .out0(new_n271));
  xnbna2aa1n03x5               g176(.a(new_n271), .b(new_n268), .c(new_n266), .out0(\s[27] ));
  norp02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .o1(new_n273));
  nanp02aa1n02x5               g178(.a(\b[27] ), .b(\a[28] ), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n274), .b(new_n273), .out0(new_n275));
  aobi12aa1n02x5               g180(.a(new_n265), .b(new_n175), .c(new_n183), .out0(new_n276));
  aoai13aa1n02x5               g181(.a(new_n242), .b(new_n231), .c(new_n216), .d(new_n228), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n264), .clkout(new_n278));
  aoai13aa1n02x5               g183(.a(new_n267), .b(new_n278), .c(new_n277), .d(new_n251), .o1(new_n279));
  norp03aa1n02x5               g184(.a(new_n279), .b(new_n276), .c(new_n269), .o1(new_n280));
  nano22aa1n02x4               g185(.a(new_n280), .b(new_n270), .c(new_n275), .out0(new_n281));
  oai112aa1n02x5               g186(.a(new_n268), .b(new_n266), .c(\b[26] ), .d(\a[27] ), .o1(new_n282));
  aoi012aa1n02x5               g187(.a(new_n275), .b(new_n282), .c(new_n270), .o1(new_n283));
  norp02aa1n02x5               g188(.a(new_n283), .b(new_n281), .o1(\s[28] ));
  nano23aa1n02x4               g189(.a(new_n269), .b(new_n273), .c(new_n274), .d(new_n270), .out0(new_n285));
  oai012aa1n02x5               g190(.a(new_n285), .b(new_n279), .c(new_n276), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n273), .b(new_n269), .c(new_n274), .o1(new_n287));
  xnrc02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .out0(new_n288));
  aoi012aa1n02x5               g193(.a(new_n288), .b(new_n286), .c(new_n287), .o1(new_n289));
  aobi12aa1n02x5               g194(.a(new_n285), .b(new_n268), .c(new_n266), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n287), .c(new_n288), .out0(new_n291));
  norp02aa1n02x5               g196(.a(new_n289), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g198(.a(new_n288), .b(new_n271), .c(new_n275), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n294), .b(new_n279), .c(new_n276), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n294), .b(new_n268), .c(new_n266), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  nano23aa1n02x4               g206(.a(new_n297), .b(new_n288), .c(new_n275), .d(new_n271), .out0(new_n302));
  aobi12aa1n02x5               g207(.a(new_n302), .b(new_n268), .c(new_n266), .out0(new_n303));
  oao003aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  nano22aa1n02x4               g210(.a(new_n303), .b(new_n304), .c(new_n305), .out0(new_n306));
  oai012aa1n02x5               g211(.a(new_n302), .b(new_n279), .c(new_n276), .o1(new_n307));
  aoi012aa1n02x5               g212(.a(new_n305), .b(new_n307), .c(new_n304), .o1(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n306), .o1(\s[31] ));
  xnrb03aa1n02x5               g214(.a(new_n145), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g215(.a(\a[3] ), .b(\b[2] ), .c(new_n145), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n147), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai112aa1n02x5               g218(.a(new_n109), .b(new_n116), .c(new_n146), .d(new_n145), .o1(new_n314));
  oaib12aa1n02x5               g219(.a(new_n314), .b(new_n118), .c(\a[5] ), .out0(new_n315));
  xnrc02aa1n02x5               g220(.a(new_n315), .b(new_n115), .out0(\s[6] ));
  oao003aa1n02x5               g221(.a(\a[6] ), .b(\b[5] ), .c(new_n315), .carry(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g223(.a(\a[7] ), .b(\b[6] ), .c(new_n317), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g225(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


