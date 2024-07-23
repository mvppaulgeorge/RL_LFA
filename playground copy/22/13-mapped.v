// Benchmark "adder" written by ABC on Thu Jul 11 12:30:20 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n185, new_n186, new_n187, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n320,
    new_n322;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(new_n97), .clkout(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  norp02aa1n02x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nona23aa1n02x4               g007(.a(new_n102), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n103));
  nanp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nona23aa1n02x4               g012(.a(new_n104), .b(new_n107), .c(new_n106), .d(new_n105), .out0(new_n108));
  norp02aa1n02x5               g013(.a(new_n108), .b(new_n103), .o1(new_n109));
  160nm_ficinv00aa1n08x5       g014(.clk(\a[4] ), .clkout(new_n110));
  160nm_ficinv00aa1n08x5       g015(.clk(\b[3] ), .clkout(new_n111));
  nanp02aa1n02x5               g016(.a(new_n111), .b(new_n110), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(new_n112), .b(new_n113), .o1(new_n114));
  xnrc02aa1n02x5               g019(.a(\b[2] ), .b(\a[3] ), .out0(new_n115));
  nanp02aa1n02x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[0] ), .b(\a[1] ), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  oai012aa1n02x5               g023(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n120), .b(new_n113), .o1(new_n121));
  oai013aa1n02x4               g026(.a(new_n121), .b(new_n115), .c(new_n119), .d(new_n114), .o1(new_n122));
  aoi112aa1n02x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  nano23aa1n02x4               g028(.a(new_n99), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n124));
  oa0012aa1n02x5               g029(.a(new_n104), .b(new_n105), .c(new_n106), .o(new_n125));
  nanp02aa1n02x5               g030(.a(new_n124), .b(new_n125), .o1(new_n126));
  nona22aa1n02x4               g031(.a(new_n126), .b(new_n123), .c(new_n99), .out0(new_n127));
  nanp02aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n97), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n127), .c(new_n109), .d(new_n122), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g039(.a(new_n122), .b(new_n109), .o1(new_n135));
  aoi112aa1n02x5               g040(.a(new_n123), .b(new_n99), .c(new_n124), .d(new_n125), .o1(new_n136));
  oai012aa1n02x5               g041(.a(new_n132), .b(new_n131), .c(new_n97), .o1(new_n137));
  nona23aa1n02x4               g042(.a(new_n132), .b(new_n128), .c(new_n97), .d(new_n131), .out0(new_n138));
  aoai13aa1n02x5               g043(.a(new_n137), .b(new_n138), .c(new_n135), .d(new_n136), .o1(new_n139));
  xorb03aa1n02x5               g044(.a(new_n139), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  norp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  nanp02aa1n02x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  aoi012aa1n02x5               g047(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n143));
  xnrb03aa1n02x5               g048(.a(new_n143), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanp02aa1n02x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nona23aa1n02x4               g051(.a(new_n146), .b(new_n142), .c(new_n141), .d(new_n145), .out0(new_n147));
  160nm_fiao0012aa1n02p5x5     g052(.a(new_n145), .b(new_n141), .c(new_n146), .o(new_n148));
  oabi12aa1n02x5               g053(.a(new_n148), .b(new_n147), .c(new_n137), .out0(new_n149));
  aoi012aa1n02x5               g054(.a(new_n127), .b(new_n122), .c(new_n109), .o1(new_n150));
  nano23aa1n02x4               g055(.a(new_n97), .b(new_n131), .c(new_n132), .d(new_n128), .out0(new_n151));
  nano23aa1n02x4               g056(.a(new_n141), .b(new_n145), .c(new_n146), .d(new_n142), .out0(new_n152));
  nano22aa1n02x4               g057(.a(new_n150), .b(new_n151), .c(new_n152), .out0(new_n153));
  xnrc02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .out0(new_n154));
  oabi12aa1n02x5               g059(.a(new_n154), .b(new_n153), .c(new_n149), .out0(new_n155));
  norb03aa1n02x5               g060(.a(new_n154), .b(new_n153), .c(new_n149), .out0(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[13] ));
  norp02aa1n02x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  160nm_ficinv00aa1n08x5       g063(.clk(new_n158), .clkout(new_n159));
  xnrc02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .out0(new_n160));
  xobna2aa1n03x5               g065(.a(new_n160), .b(new_n155), .c(new_n159), .out0(\s[14] ));
  norp02aa1n02x5               g066(.a(new_n160), .b(new_n154), .o1(new_n162));
  nano32aa1n02x4               g067(.a(new_n150), .b(new_n162), .c(new_n151), .d(new_n152), .out0(new_n163));
  oao003aa1n02x5               g068(.a(\a[14] ), .b(\b[13] ), .c(new_n159), .carry(new_n164));
  160nm_ficinv00aa1n08x5       g069(.clk(new_n164), .clkout(new_n165));
  aoi012aa1n02x5               g070(.a(new_n165), .b(new_n149), .c(new_n162), .o1(new_n166));
  xnrc02aa1n02x5               g071(.a(\b[14] ), .b(\a[15] ), .out0(new_n167));
  160nm_ficinv00aa1n08x5       g072(.clk(new_n167), .clkout(new_n168));
  oaib12aa1n02x5               g073(.a(new_n168), .b(new_n163), .c(new_n166), .out0(new_n169));
  nano22aa1n02x4               g074(.a(new_n163), .b(new_n166), .c(new_n167), .out0(new_n170));
  norb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(\s[15] ));
  norp02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n172), .clkout(new_n173));
  xnrc02aa1n02x5               g078(.a(\b[15] ), .b(\a[16] ), .out0(new_n174));
  xobna2aa1n03x5               g079(.a(new_n174), .b(new_n169), .c(new_n173), .out0(\s[16] ));
  nanp02aa1n02x5               g080(.a(new_n152), .b(new_n151), .o1(new_n176));
  norp02aa1n02x5               g081(.a(new_n174), .b(new_n167), .o1(new_n177));
  nano22aa1n02x4               g082(.a(new_n176), .b(new_n162), .c(new_n177), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n127), .c(new_n109), .d(new_n122), .o1(new_n179));
  aoai13aa1n02x5               g084(.a(new_n177), .b(new_n165), .c(new_n149), .d(new_n162), .o1(new_n180));
  oao003aa1n02x5               g085(.a(\a[16] ), .b(\b[15] ), .c(new_n173), .carry(new_n181));
  nanp03aa1n02x5               g086(.a(new_n179), .b(new_n180), .c(new_n181), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g088(.clk(\a[18] ), .clkout(new_n184));
  160nm_ficinv00aa1n08x5       g089(.clk(\a[17] ), .clkout(new_n185));
  160nm_ficinv00aa1n08x5       g090(.clk(\b[16] ), .clkout(new_n186));
  oaoi03aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n182), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n184), .out0(\s[18] ));
  nona23aa1n02x4               g093(.a(new_n177), .b(new_n162), .c(new_n138), .d(new_n147), .out0(new_n189));
  aoi012aa1n02x5               g094(.a(new_n189), .b(new_n135), .c(new_n136), .o1(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(new_n137), .clkout(new_n191));
  aoai13aa1n02x5               g096(.a(new_n162), .b(new_n148), .c(new_n152), .d(new_n191), .o1(new_n192));
  160nm_ficinv00aa1n08x5       g097(.clk(new_n177), .clkout(new_n193));
  aoai13aa1n02x5               g098(.a(new_n181), .b(new_n193), .c(new_n192), .d(new_n164), .o1(new_n194));
  xroi22aa1d04x5               g099(.a(new_n185), .b(\b[16] ), .c(new_n184), .d(\b[17] ), .out0(new_n195));
  oai012aa1n02x5               g100(.a(new_n195), .b(new_n194), .c(new_n190), .o1(new_n196));
  oai022aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n197));
  oaib12aa1n02x5               g102(.a(new_n197), .b(new_n184), .c(\b[17] ), .out0(new_n198));
  norp02aa1n02x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanp02aa1n02x5               g104(.a(\b[18] ), .b(\a[19] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  xobna2aa1n03x5               g106(.a(new_n201), .b(new_n196), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  160nm_ficinv00aa1n08x5       g108(.clk(new_n199), .clkout(new_n204));
  aoi012aa1n02x5               g109(.a(new_n201), .b(new_n196), .c(new_n198), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(new_n208));
  nano22aa1n02x4               g113(.a(new_n205), .b(new_n204), .c(new_n208), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n186), .b(new_n185), .o1(new_n210));
  oaoi03aa1n02x5               g115(.a(\a[18] ), .b(\b[17] ), .c(new_n210), .o1(new_n211));
  oaoi13aa1n02x5               g116(.a(new_n211), .b(new_n195), .c(new_n194), .d(new_n190), .o1(new_n212));
  oaoi13aa1n02x5               g117(.a(new_n208), .b(new_n204), .c(new_n212), .d(new_n201), .o1(new_n213));
  norp02aa1n02x5               g118(.a(new_n213), .b(new_n209), .o1(\s[20] ));
  nano23aa1n02x4               g119(.a(new_n199), .b(new_n206), .c(new_n207), .d(new_n200), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(new_n195), .b(new_n215), .o1(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(new_n216), .clkout(new_n217));
  oai012aa1n02x5               g122(.a(new_n217), .b(new_n194), .c(new_n190), .o1(new_n218));
  nona23aa1n02x4               g123(.a(new_n207), .b(new_n200), .c(new_n199), .d(new_n206), .out0(new_n219));
  aoi012aa1n02x5               g124(.a(new_n206), .b(new_n199), .c(new_n207), .o1(new_n220));
  oai012aa1n02x5               g125(.a(new_n220), .b(new_n219), .c(new_n198), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n221), .clkout(new_n222));
  norp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n218), .c(new_n222), .out0(\s[21] ));
  160nm_ficinv00aa1n08x5       g131(.clk(new_n223), .clkout(new_n227));
  aobi12aa1n02x5               g132(.a(new_n225), .b(new_n218), .c(new_n222), .out0(new_n228));
  xnrc02aa1n02x5               g133(.a(\b[21] ), .b(\a[22] ), .out0(new_n229));
  nano22aa1n02x4               g134(.a(new_n228), .b(new_n227), .c(new_n229), .out0(new_n230));
  aoai13aa1n02x5               g135(.a(new_n225), .b(new_n221), .c(new_n182), .d(new_n217), .o1(new_n231));
  aoi012aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .o1(new_n232));
  norp02aa1n02x5               g137(.a(new_n232), .b(new_n230), .o1(\s[22] ));
  nano22aa1n02x4               g138(.a(new_n229), .b(new_n227), .c(new_n224), .out0(new_n234));
  and003aa1n02x5               g139(.a(new_n195), .b(new_n234), .c(new_n215), .o(new_n235));
  oai012aa1n02x5               g140(.a(new_n235), .b(new_n194), .c(new_n190), .o1(new_n236));
  oao003aa1n02x5               g141(.a(\a[22] ), .b(\b[21] ), .c(new_n227), .carry(new_n237));
  160nm_ficinv00aa1n08x5       g142(.clk(new_n237), .clkout(new_n238));
  aoi012aa1n02x5               g143(.a(new_n238), .b(new_n221), .c(new_n234), .o1(new_n239));
  xnrc02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .out0(new_n240));
  160nm_ficinv00aa1n08x5       g145(.clk(new_n240), .clkout(new_n241));
  xnbna2aa1n03x5               g146(.a(new_n241), .b(new_n236), .c(new_n239), .out0(\s[23] ));
  norp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  160nm_ficinv00aa1n08x5       g148(.clk(new_n243), .clkout(new_n244));
  aoi012aa1n02x5               g149(.a(new_n240), .b(new_n236), .c(new_n239), .o1(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  nano22aa1n02x4               g151(.a(new_n245), .b(new_n244), .c(new_n246), .out0(new_n247));
  160nm_ficinv00aa1n08x5       g152(.clk(new_n239), .clkout(new_n248));
  oaoi13aa1n02x5               g153(.a(new_n248), .b(new_n235), .c(new_n194), .d(new_n190), .o1(new_n249));
  oaoi13aa1n02x5               g154(.a(new_n246), .b(new_n244), .c(new_n249), .d(new_n240), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n247), .o1(\s[24] ));
  norp02aa1n02x5               g156(.a(new_n246), .b(new_n240), .o1(new_n252));
  nano22aa1n02x4               g157(.a(new_n216), .b(new_n234), .c(new_n252), .out0(new_n253));
  160nm_ficinv00aa1n08x5       g158(.clk(new_n220), .clkout(new_n254));
  aoai13aa1n02x5               g159(.a(new_n234), .b(new_n254), .c(new_n215), .d(new_n211), .o1(new_n255));
  160nm_ficinv00aa1n08x5       g160(.clk(new_n252), .clkout(new_n256));
  oao003aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .carry(new_n257));
  aoai13aa1n02x5               g162(.a(new_n257), .b(new_n256), .c(new_n255), .d(new_n237), .o1(new_n258));
  oaoi13aa1n02x5               g163(.a(new_n258), .b(new_n253), .c(new_n194), .d(new_n190), .o1(new_n259));
  xnrb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n261), .clkout(new_n262));
  oai012aa1n02x5               g167(.a(new_n253), .b(new_n194), .c(new_n190), .o1(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .out0(new_n264));
  aoib12aa1n02x5               g169(.a(new_n264), .b(new_n263), .c(new_n258), .out0(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[25] ), .b(\a[26] ), .out0(new_n266));
  nano22aa1n02x4               g171(.a(new_n265), .b(new_n262), .c(new_n266), .out0(new_n267));
  oaoi13aa1n02x5               g172(.a(new_n266), .b(new_n262), .c(new_n259), .d(new_n264), .o1(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n267), .o1(\s[26] ));
  norp02aa1n02x5               g174(.a(new_n266), .b(new_n264), .o1(new_n270));
  nano32aa1n02x4               g175(.a(new_n216), .b(new_n270), .c(new_n234), .d(new_n252), .out0(new_n271));
  oai012aa1n02x5               g176(.a(new_n271), .b(new_n194), .c(new_n190), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n273));
  aobi12aa1n02x5               g178(.a(new_n273), .b(new_n258), .c(new_n270), .out0(new_n274));
  xorc02aa1n02x5               g179(.a(\a[27] ), .b(\b[26] ), .out0(new_n275));
  xnbna2aa1n03x5               g180(.a(new_n275), .b(new_n272), .c(new_n274), .out0(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  160nm_ficinv00aa1n08x5       g182(.clk(new_n277), .clkout(new_n278));
  aobi12aa1n02x5               g183(.a(new_n275), .b(new_n272), .c(new_n274), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n279), .b(new_n278), .c(new_n280), .out0(new_n281));
  aoai13aa1n02x5               g186(.a(new_n252), .b(new_n238), .c(new_n221), .d(new_n234), .o1(new_n282));
  160nm_ficinv00aa1n08x5       g187(.clk(new_n270), .clkout(new_n283));
  aoai13aa1n02x5               g188(.a(new_n273), .b(new_n283), .c(new_n282), .d(new_n257), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n275), .b(new_n284), .c(new_n182), .d(new_n271), .o1(new_n285));
  aoi012aa1n02x5               g190(.a(new_n280), .b(new_n285), .c(new_n278), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n281), .o1(\s[28] ));
  norb02aa1n02x5               g192(.a(new_n275), .b(new_n280), .out0(new_n288));
  aobi12aa1n02x5               g193(.a(new_n288), .b(new_n272), .c(new_n274), .out0(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  nano22aa1n02x4               g196(.a(new_n289), .b(new_n290), .c(new_n291), .out0(new_n292));
  aoai13aa1n02x5               g197(.a(new_n288), .b(new_n284), .c(new_n182), .d(new_n271), .o1(new_n293));
  aoi012aa1n02x5               g198(.a(new_n291), .b(new_n293), .c(new_n290), .o1(new_n294));
  norp02aa1n02x5               g199(.a(new_n294), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n117), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n275), .b(new_n291), .c(new_n280), .out0(new_n297));
  aobi12aa1n02x5               g202(.a(new_n297), .b(new_n272), .c(new_n274), .out0(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  nano22aa1n02x4               g205(.a(new_n298), .b(new_n299), .c(new_n300), .out0(new_n301));
  aoai13aa1n02x5               g206(.a(new_n297), .b(new_n284), .c(new_n182), .d(new_n271), .o1(new_n302));
  aoi012aa1n02x5               g207(.a(new_n300), .b(new_n302), .c(new_n299), .o1(new_n303));
  norp02aa1n02x5               g208(.a(new_n303), .b(new_n301), .o1(\s[30] ));
  norb02aa1n02x5               g209(.a(new_n297), .b(new_n300), .out0(new_n305));
  aobi12aa1n02x5               g210(.a(new_n305), .b(new_n272), .c(new_n274), .out0(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  xnrc02aa1n02x5               g212(.a(\b[30] ), .b(\a[31] ), .out0(new_n308));
  nano22aa1n02x4               g213(.a(new_n306), .b(new_n307), .c(new_n308), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n305), .b(new_n284), .c(new_n182), .d(new_n271), .o1(new_n310));
  aoi012aa1n02x5               g215(.a(new_n308), .b(new_n310), .c(new_n307), .o1(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n309), .o1(\s[31] ));
  xnrb03aa1n02x5               g217(.a(new_n119), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n119), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g221(.a(new_n106), .b(new_n122), .c(new_n107), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  160nm_ficinv00aa1n08x5       g223(.clk(new_n108), .clkout(new_n319));
  160nm_fiao0012aa1n02p5x5     g224(.a(new_n125), .b(new_n122), .c(new_n319), .o(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g226(.a(new_n101), .b(new_n320), .c(new_n102), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g228(.a(new_n129), .b(new_n135), .c(new_n136), .out0(\s[9] ));
endmodule


